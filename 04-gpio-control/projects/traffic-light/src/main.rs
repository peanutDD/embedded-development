#![no_std]
#![no_main]

use panic_halt as _;
use rtt_target::{rprintln, rtt_init_print};

use cortex_m::delay::Delay;
use cortex_m_rt::entry;
use stm32f4xx_hal::{
    gpio::{Output, Pin, PushPull},
    pac,
    prelude::*,
    timer::{CounterUs, Event},
};

use heapless::Vec;

// 系统配置常量
const SYSTEM_CLOCK_HZ: u32 = 168_000_000;
const TIMER_FREQUENCY_HZ: u32 = 1000; // 1kHz定时器

// 交通灯时序配置（毫秒）
const GREEN_DURATION_MS: u32 = 5000;    // 绿灯5秒
const YELLOW_DURATION_MS: u32 = 2000;   // 黄灯2秒
const RED_DURATION_MS: u32 = 3000;      // 红灯3秒
const PEDESTRIAN_WAIT_MS: u32 = 1000;   // 行人等待1秒
const PEDESTRIAN_CROSS_MS: u32 = 8000;  // 行人通行8秒

// 夜间模式配置
const NIGHT_MODE_BLINK_MS: u32 = 1000;  // 夜间闪烁间隔
const NIGHT_MODE_START_HOUR: u8 = 22;   // 夜间模式开始时间
const NIGHT_MODE_END_HOUR: u8 = 6;      // 夜间模式结束时间

// 紧急模式配置
const EMERGENCY_BLINK_MS: u32 = 500;    // 紧急闪烁间隔

// 类型别名
type LedPin = Pin<'A', 0, Output<PushPull>>;
type ButtonPin = Pin<'C', 13>;

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("智能交通灯系统启动");

    // 初始化外设
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // 配置系统时钟
    let rcc = dp.RCC.constrain();
    let clocks = rcc
        .cfgr
        .use_hse(8.MHz())
        .sysclk(168.MHz())
        .require_pll48clk()
        .freeze();

    rprintln!("系统时钟配置完成: SYSCLK={}MHz", clocks.sysclk().raw() / 1_000_000);

    // 配置GPIO
    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();

    // 主路交通灯 (PA0-PA2)
    let main_red = gpioa.pa0.into_push_pull_output();
    let main_yellow = gpioa.pa1.into_push_pull_output();
    let main_green = gpioa.pa2.into_push_pull_output();

    // 辅路交通灯 (PA3-PA5)
    let side_red = gpioa.pa3.into_push_pull_output();
    let side_yellow = gpioa.pa4.into_push_pull_output();
    let side_green = gpioa.pa5.into_push_pull_output();

    // 行人信号灯 (PB0-PB1)
    let pedestrian_red = gpiob.pb0.into_push_pull_output();
    let pedestrian_green = gpiob.pb1.into_push_pull_output();

    // 按钮输入 (PC13: 行人按钮, PC14: 紧急按钮, PC15: 夜间模式)
    let pedestrian_button = gpioc.pc13.into_pull_up_input();
    let emergency_button = gpioc.pc14.into_pull_up_input();
    let night_mode_button = gpioc.pc15.into_pull_up_input();

    // 状态指示LED (PB2-PB4)
    let status_normal = gpiob.pb2.into_push_pull_output();
    let status_emergency = gpiob.pb3.into_push_pull_output();
    let status_night = gpiob.pb4.into_push_pull_output();

    // 配置定时器
    let mut timer = dp.TIM2.counter_us(&clocks);
    timer.start(1000.micros()).unwrap(); // 1ms定时器

    // 创建延时对象
    let mut delay = Delay::new(cp.SYST, clocks.sysclk().raw());

    // 创建交通灯控制器
    let mut traffic_controller = TrafficLightController::new(
        MainTrafficLights {
            red: main_red,
            yellow: main_yellow,
            green: main_green,
        },
        SideTrafficLights {
            red: side_red,
            yellow: side_yellow,
            green: side_green,
        },
        PedestrianLights {
            red: pedestrian_red,
            green: pedestrian_green,
        },
        StatusLights {
            normal: status_normal,
            emergency: status_emergency,
            night: status_night,
        },
    );

    // 创建按钮管理器
    let mut button_manager = ButtonManager::new(
        pedestrian_button,
        emergency_button,
        night_mode_button,
    );

    // 创建系统状态管理器
    let mut system_state = SystemState::new();

    rprintln!("交通灯系统初始化完成");
    rprintln!("开始交通灯控制循环");

    let mut last_time = 0u32;
    let mut cycle_count = 0u32;

    loop {
        // 检查定时器
        if timer.wait().is_ok() {
            last_time = last_time.wrapping_add(1);
        }

        // 更新按钮状态
        button_manager.update();

        // 处理按钮事件
        if button_manager.is_pedestrian_pressed() {
            rprintln!("行人按钮按下");
            system_state.request_pedestrian_crossing();
        }

        if button_manager.is_emergency_pressed() {
            rprintln!("紧急按钮按下");
            system_state.toggle_emergency_mode();
        }

        if button_manager.is_night_mode_pressed() {
            rprintln!("夜间模式按钮按下");
            system_state.toggle_night_mode();
        }

        // 更新系统状态
        system_state.update(last_time);

        // 根据当前模式控制交通灯
        match system_state.get_current_mode() {
            SystemMode::Normal => {
                traffic_controller.run_normal_cycle(&mut system_state, last_time);
            }
            SystemMode::Emergency => {
                traffic_controller.run_emergency_mode(last_time);
            }
            SystemMode::Night => {
                traffic_controller.run_night_mode(last_time);
            }
            SystemMode::Maintenance => {
                traffic_controller.run_maintenance_mode(last_time);
            }
        }

        // 更新状态指示灯
        traffic_controller.update_status_lights(&system_state);

        // 性能统计
        if last_time % 10000 == 0 {
            cycle_count += 1;
            rprintln!("运行统计: 周期={}, 时间={}ms, 模式={:?}", 
                     cycle_count, last_time, system_state.get_current_mode());
            
            let stats = system_state.get_statistics();
            rprintln!("统计信息: 正常周期={}, 行人请求={}, 紧急事件={}", 
                     stats.normal_cycles, stats.pedestrian_requests, stats.emergency_events);
        }

        // 短暂延时以避免过度占用CPU
        delay.delay_us(100);
    }
}

// 主路交通灯结构体
struct MainTrafficLights {
    red: Pin<'A', 0, Output<PushPull>>,
    yellow: Pin<'A', 1, Output<PushPull>>,
    green: Pin<'A', 2, Output<PushPull>>,
}

impl MainTrafficLights {
    fn set_state(&mut self, state: TrafficLightState) {
        match state {
            TrafficLightState::Red => {
                self.red.set_high();
                self.yellow.set_low();
                self.green.set_low();
            }
            TrafficLightState::Yellow => {
                self.red.set_low();
                self.yellow.set_high();
                self.green.set_low();
            }
            TrafficLightState::Green => {
                self.red.set_low();
                self.yellow.set_low();
                self.green.set_high();
            }
            TrafficLightState::Off => {
                self.red.set_low();
                self.yellow.set_low();
                self.green.set_low();
            }
            TrafficLightState::RedYellow => {
                self.red.set_high();
                self.yellow.set_high();
                self.green.set_low();
            }
        }
    }

    fn blink_yellow(&mut self, on: bool) {
        if on {
            self.red.set_low();
            self.yellow.set_high();
            self.green.set_low();
        } else {
            self.red.set_low();
            self.yellow.set_low();
            self.green.set_low();
        }
    }

    fn all_off(&mut self) {
        self.red.set_low();
        self.yellow.set_low();
        self.green.set_low();
    }
}

// 辅路交通灯结构体
struct SideTrafficLights {
    red: Pin<'A', 3, Output<PushPull>>,
    yellow: Pin<'A', 4, Output<PushPull>>,
    green: Pin<'A', 5, Output<PushPull>>,
}

impl SideTrafficLights {
    fn set_state(&mut self, state: TrafficLightState) {
        match state {
            TrafficLightState::Red => {
                self.red.set_high();
                self.yellow.set_low();
                self.green.set_low();
            }
            TrafficLightState::Yellow => {
                self.red.set_low();
                self.yellow.set_high();
                self.green.set_low();
            }
            TrafficLightState::Green => {
                self.red.set_low();
                self.yellow.set_low();
                self.green.set_high();
            }
            TrafficLightState::Off => {
                self.red.set_low();
                self.yellow.set_low();
                self.green.set_low();
            }
            TrafficLightState::RedYellow => {
                self.red.set_high();
                self.yellow.set_high();
                self.green.set_low();
            }
        }
    }

    fn blink_yellow(&mut self, on: bool) {
        if on {
            self.red.set_low();
            self.yellow.set_high();
            self.green.set_low();
        } else {
            self.red.set_low();
            self.yellow.set_low();
            self.green.set_low();
        }
    }

    fn all_off(&mut self) {
        self.red.set_low();
        self.yellow.set_low();
        self.green.set_low();
    }
}

// 行人信号灯结构体
struct PedestrianLights {
    red: Pin<'B', 0, Output<PushPull>>,
    green: Pin<'B', 1, Output<PushPull>>,
}

impl PedestrianLights {
    fn set_red(&mut self) {
        self.red.set_high();
        self.green.set_low();
    }

    fn set_green(&mut self) {
        self.red.set_low();
        self.green.set_high();
    }

    fn blink_green(&mut self, on: bool) {
        self.red.set_low();
        if on {
            self.green.set_high();
        } else {
            self.green.set_low();
        }
    }

    fn all_off(&mut self) {
        self.red.set_low();
        self.green.set_low();
    }
}

// 状态指示灯结构体
struct StatusLights {
    normal: Pin<'B', 2, Output<PushPull>>,
    emergency: Pin<'B', 3, Output<PushPull>>,
    night: Pin<'B', 4, Output<PushPull>>,
}

impl StatusLights {
    fn set_normal(&mut self) {
        self.normal.set_high();
        self.emergency.set_low();
        self.night.set_low();
    }

    fn set_emergency(&mut self) {
        self.normal.set_low();
        self.emergency.set_high();
        self.night.set_low();
    }

    fn set_night(&mut self) {
        self.normal.set_low();
        self.emergency.set_low();
        self.night.set_high();
    }

    fn blink_emergency(&mut self, on: bool) {
        self.normal.set_low();
        self.night.set_low();
        if on {
            self.emergency.set_high();
        } else {
            self.emergency.set_low();
        }
    }

    fn all_off(&mut self) {
        self.normal.set_low();
        self.emergency.set_low();
        self.night.set_low();
    }
}

// 交通灯状态枚举
#[derive(Debug, Clone, Copy, PartialEq)]
enum TrafficLightState {
    Red,
    Yellow,
    Green,
    RedYellow,  // 红黄同时亮（某些国家使用）
    Off,
}

// 系统模式枚举
#[derive(Debug, Clone, Copy, PartialEq)]
enum SystemMode {
    Normal,      // 正常模式
    Emergency,   // 紧急模式
    Night,       // 夜间模式
    Maintenance, // 维护模式
}

// 正常模式状态枚举
#[derive(Debug, Clone, Copy, PartialEq)]
enum NormalState {
    MainGreen,      // 主路绿灯
    MainYellow,     // 主路黄灯
    SideGreen,      // 辅路绿灯
    SideYellow,     // 辅路黄灯
    PedestrianWait, // 行人等待
    PedestrianCross,// 行人通行
}

// 交通灯控制器
struct TrafficLightController {
    main_lights: MainTrafficLights,
    side_lights: SideTrafficLights,
    pedestrian_lights: PedestrianLights,
    status_lights: StatusLights,
    current_state: NormalState,
    state_start_time: u32,
    blink_state: bool,
    last_blink_time: u32,
}

impl TrafficLightController {
    fn new(
        main_lights: MainTrafficLights,
        side_lights: SideTrafficLights,
        pedestrian_lights: PedestrianLights,
        status_lights: StatusLights,
    ) -> Self {
        Self {
            main_lights,
            side_lights,
            pedestrian_lights,
            status_lights,
            current_state: NormalState::MainGreen,
            state_start_time: 0,
            blink_state: false,
            last_blink_time: 0,
        }
    }

    fn run_normal_cycle(&mut self, system_state: &mut SystemState, current_time: u32) {
        let state_duration = current_time.wrapping_sub(self.state_start_time);
        let mut should_transition = false;
        let mut next_state = self.current_state;

        match self.current_state {
            NormalState::MainGreen => {
                self.main_lights.set_state(TrafficLightState::Green);
                self.side_lights.set_state(TrafficLightState::Red);
                self.pedestrian_lights.set_red();

                if state_duration >= GREEN_DURATION_MS || system_state.has_pedestrian_request() {
                    should_transition = true;
                    next_state = NormalState::MainYellow;
                }
            }
            NormalState::MainYellow => {
                self.main_lights.set_state(TrafficLightState::Yellow);
                self.side_lights.set_state(TrafficLightState::Red);
                self.pedestrian_lights.set_red();

                if state_duration >= YELLOW_DURATION_MS {
                    should_transition = true;
                    if system_state.has_pedestrian_request() {
                        next_state = NormalState::PedestrianWait;
                        system_state.clear_pedestrian_request();
                    } else {
                        next_state = NormalState::SideGreen;
                    }
                }
            }
            NormalState::SideGreen => {
                self.main_lights.set_state(TrafficLightState::Red);
                self.side_lights.set_state(TrafficLightState::Green);
                self.pedestrian_lights.set_red();

                if state_duration >= GREEN_DURATION_MS {
                    should_transition = true;
                    next_state = NormalState::SideYellow;
                }
            }
            NormalState::SideYellow => {
                self.main_lights.set_state(TrafficLightState::Red);
                self.side_lights.set_state(TrafficLightState::Yellow);
                self.pedestrian_lights.set_red();

                if state_duration >= YELLOW_DURATION_MS {
                    should_transition = true;
                    next_state = NormalState::MainGreen;
                }
            }
            NormalState::PedestrianWait => {
                self.main_lights.set_state(TrafficLightState::Red);
                self.side_lights.set_state(TrafficLightState::Red);
                self.pedestrian_lights.set_red();

                if state_duration >= PEDESTRIAN_WAIT_MS {
                    should_transition = true;
                    next_state = NormalState::PedestrianCross;
                }
            }
            NormalState::PedestrianCross => {
                self.main_lights.set_state(TrafficLightState::Red);
                self.side_lights.set_state(TrafficLightState::Red);
                
                // 行人绿灯闪烁提醒
                if state_duration > PEDESTRIAN_CROSS_MS - 3000 {
                    self.update_blink(current_time, 500);
                    self.pedestrian_lights.blink_green(self.blink_state);
                } else {
                    self.pedestrian_lights.set_green();
                }

                if state_duration >= PEDESTRIAN_CROSS_MS {
                    should_transition = true;
                    next_state = NormalState::MainGreen;
                }
            }
        }

        if should_transition {
            rprintln!("状态转换: {:?} -> {:?}", self.current_state, next_state);
            self.current_state = next_state;
            self.state_start_time = current_time;
            system_state.increment_normal_cycles();
        }
    }

    fn run_emergency_mode(&mut self, current_time: u32) {
        self.update_blink(current_time, EMERGENCY_BLINK_MS);
        
        // 所有交通灯闪烁黄灯
        self.main_lights.blink_yellow(self.blink_state);
        self.side_lights.blink_yellow(self.blink_state);
        self.pedestrian_lights.all_off();
    }

    fn run_night_mode(&mut self, current_time: u32) {
        self.update_blink(current_time, NIGHT_MODE_BLINK_MS);
        
        // 主路闪烁黄灯，辅路红灯
        self.main_lights.blink_yellow(self.blink_state);
        self.side_lights.set_state(TrafficLightState::Red);
        self.pedestrian_lights.set_red();
    }

    fn run_maintenance_mode(&mut self, current_time: u32) {
        self.update_blink(current_time, 2000);
        
        if self.blink_state {
            self.main_lights.set_state(TrafficLightState::Red);
            self.side_lights.set_state(TrafficLightState::Red);
            self.pedestrian_lights.set_red();
        } else {
            self.main_lights.all_off();
            self.side_lights.all_off();
            self.pedestrian_lights.all_off();
        }
    }

    fn update_status_lights(&mut self, system_state: &SystemState) {
        match system_state.get_current_mode() {
            SystemMode::Normal => self.status_lights.set_normal(),
            SystemMode::Emergency => {
                self.status_lights.blink_emergency(self.blink_state);
            }
            SystemMode::Night => self.status_lights.set_night(),
            SystemMode::Maintenance => self.status_lights.all_off(),
        }
    }

    fn update_blink(&mut self, current_time: u32, interval_ms: u32) {
        if current_time.wrapping_sub(self.last_blink_time) >= interval_ms {
            self.blink_state = !self.blink_state;
            self.last_blink_time = current_time;
        }
    }
}

// 按钮管理器
struct ButtonManager {
    pedestrian_button: Pin<'C', 13>,
    emergency_button: Pin<'C', 14>,
    night_mode_button: Pin<'C', 15>,
    pedestrian_pressed: bool,
    emergency_pressed: bool,
    night_mode_pressed: bool,
    pedestrian_last_state: bool,
    emergency_last_state: bool,
    night_mode_last_state: bool,
}

impl ButtonManager {
    fn new(
        pedestrian_button: Pin<'C', 13>,
        emergency_button: Pin<'C', 14>,
        night_mode_button: Pin<'C', 15>,
    ) -> Self {
        Self {
            pedestrian_button,
            emergency_button,
            night_mode_button,
            pedestrian_pressed: false,
            emergency_pressed: false,
            night_mode_pressed: false,
            pedestrian_last_state: true, // 上拉输入，默认高电平
            emergency_last_state: true,
            night_mode_last_state: true,
        }
    }

    fn update(&mut self) {
        // 检测按钮按下（下降沿）
        let pedestrian_current = self.pedestrian_button.is_high();
        let emergency_current = self.emergency_button.is_high();
        let night_mode_current = self.night_mode_button.is_high();

        self.pedestrian_pressed = self.pedestrian_last_state && !pedestrian_current;
        self.emergency_pressed = self.emergency_last_state && !emergency_current;
        self.night_mode_pressed = self.night_mode_last_state && !night_mode_current;

        self.pedestrian_last_state = pedestrian_current;
        self.emergency_last_state = emergency_current;
        self.night_mode_last_state = night_mode_current;
    }

    fn is_pedestrian_pressed(&mut self) -> bool {
        let pressed = self.pedestrian_pressed;
        self.pedestrian_pressed = false; // 清除标志
        pressed
    }

    fn is_emergency_pressed(&mut self) -> bool {
        let pressed = self.emergency_pressed;
        self.emergency_pressed = false;
        pressed
    }

    fn is_night_mode_pressed(&mut self) -> bool {
        let pressed = self.night_mode_pressed;
        self.night_mode_pressed = false;
        pressed
    }
}

// 系统状态管理器
struct SystemState {
    current_mode: SystemMode,
    pedestrian_request: bool,
    statistics: SystemStatistics,
    mode_start_time: u32,
}

impl SystemState {
    fn new() -> Self {
        Self {
            current_mode: SystemMode::Normal,
            pedestrian_request: false,
            statistics: SystemStatistics::new(),
            mode_start_time: 0,
        }
    }

    fn update(&mut self, current_time: u32) {
        // 可以在这里添加自动模式切换逻辑
        // 例如：根据时间自动切换到夜间模式
    }

    fn get_current_mode(&self) -> SystemMode {
        self.current_mode
    }

    fn request_pedestrian_crossing(&mut self) {
        self.pedestrian_request = true;
        self.statistics.pedestrian_requests += 1;
    }

    fn has_pedestrian_request(&self) -> bool {
        self.pedestrian_request
    }

    fn clear_pedestrian_request(&mut self) {
        self.pedestrian_request = false;
    }

    fn toggle_emergency_mode(&mut self) {
        match self.current_mode {
            SystemMode::Emergency => {
                self.current_mode = SystemMode::Normal;
                rprintln!("退出紧急模式");
            }
            _ => {
                self.current_mode = SystemMode::Emergency;
                self.statistics.emergency_events += 1;
                rprintln!("进入紧急模式");
            }
        }
    }

    fn toggle_night_mode(&mut self) {
        match self.current_mode {
            SystemMode::Night => {
                self.current_mode = SystemMode::Normal;
                rprintln!("退出夜间模式");
            }
            _ => {
                self.current_mode = SystemMode::Night;
                rprintln!("进入夜间模式");
            }
        }
    }

    fn increment_normal_cycles(&mut self) {
        self.statistics.normal_cycles += 1;
    }

    fn get_statistics(&self) -> &SystemStatistics {
        &self.statistics
    }
}

// 系统统计信息
struct SystemStatistics {
    normal_cycles: u32,
    pedestrian_requests: u32,
    emergency_events: u32,
    mode_switches: u32,
}

impl SystemStatistics {
    fn new() -> Self {
        Self {
            normal_cycles: 0,
            pedestrian_requests: 0,
            emergency_events: 0,
            mode_switches: 0,
        }
    }

    fn reset(&mut self) {
        self.normal_cycles = 0;
        self.pedestrian_requests = 0;
        self.emergency_events = 0;
        self.mode_switches = 0;
    }
}

// 测试模块
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_traffic_light_state_transitions() {
        // 这里可以添加状态转换的单元测试
        assert_eq!(TrafficLightState::Red, TrafficLightState::Red);
    }

    #[test]
    fn test_system_statistics() {
        let mut stats = SystemStatistics::new();
        assert_eq!(stats.normal_cycles, 0);
        
        stats.normal_cycles += 1;
        assert_eq!(stats.normal_cycles, 1);
        
        stats.reset();
        assert_eq!(stats.normal_cycles, 0);
    }
}