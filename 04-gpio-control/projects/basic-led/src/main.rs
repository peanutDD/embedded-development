#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;
use rtt_target::{rprintln, rtt_init_print};
use stm32f4xx_hal::{
  gpio::{Output, Pin, PushPull},
  pac,
  prelude::*,
  timer::Timer,
};

// 导入LED控制库
use basic_led::{BasicLed, BlinkPattern, IndicatorState, LedBlinker, LedController, LedIndicator};

// LED引脚类型别名
type LedPin = Pin<'C', 13, Output<PushPull>>;

// 简单的延时提供者/// 简单延时实现
struct SimpleDelay {
  timer: Option<Timer<pac::TIM3>>,
}

impl SimpleDelay {
  fn new(timer: Timer<pac::TIM3>) -> Self {
    Self { timer: Some(timer) }
  }
}

impl embedded_hal::delay::DelayNs for SimpleDelay {
  fn delay_ns(&mut self, ns: u32) {
    // 简化实现，将纳秒转换为毫秒
    let ms = (ns / 1_000_000).max(1);
    self.delay_ms(ms);
  }

  fn delay_us(&mut self, us: u32) {
    // 简化实现，将微秒转换为毫秒
    let ms = (us / 1000).max(1);
    self.delay_ms(ms);
  }

  fn delay_ms(&mut self, ms: u32) {
    for _ in 0..ms {
      // 使用定时器进行1ms延时
      if let Some(timer) = self.timer.take() {
        let mut counter = timer.counter_hz();
        counter.start(1000.Hz()).unwrap();
        nb::block!(counter.wait()).unwrap();
        self.timer = Some(counter.release());
      }
    }
  }
}

#[entry]
fn main() -> ! {
  // 初始化RTT调试输出
  rtt_init_print!();
  rprintln!("基础LED控制程序启动 - 使用LED控制库");

  // 获取设备外设
  let dp = pac::Peripherals::take().unwrap();
  let cp = cortex_m::Peripherals::take().unwrap();

  // 配置时钟
  let rcc = dp.RCC.constrain();
  let clocks = rcc
    .cfgr
    .use_hse(8.MHz()) // 使用外部8MHz晶振
    .sysclk(168.MHz()) // 系统时钟168MHz
    .freeze();

  // 配置GPIO
  let gpioc = dp.GPIOC.split();
  let led_pin = gpioc.pc13.into_push_pull_output();

  // 创建LED控制器 (STM32F4的板载LED通常是低电平有效)
  let mut led = BasicLed::active_low(led_pin);

  // 配置定时器用于延时
  let delay_timer = Timer::new(dp.TIM3, &clocks);
  let delay = SimpleDelay::new(delay_timer);

  // 创建LED闪烁器
  let mut blinker = LedBlinker::new(led, delay, BlinkPattern::Uniform);

  rprintln!("开始LED闪烁演示");

  // 演示1: 基本LED控制
  rprintln!("演示1: 基本LED开关控制");
  for i in 0..5 {
    rprintln!("  第{}次: 开启LED", i + 1);
    blinker.led_mut().turn_on().unwrap();
    cortex_m::asm::delay(8_000_000); // 简单延时

    rprintln!("  第{}次: 关闭LED", i + 1);
    blinker.led_mut().turn_off().unwrap();
    cortex_m::asm::delay(8_000_000);
  }

  // 演示2: LED切换
  rprintln!("演示2: LED状态切换");
  for i in 0..10 {
    rprintln!(
      "  切换{}次, LED状态: {}",
      i + 1,
      if blinker.led().is_on() {
        "开启"
      } else {
        "关闭"
      }
    );
    blinker.led_mut().toggle().unwrap();
    cortex_m::asm::delay(4_000_000);
  }

  // 演示3: 不同闪烁模式
  rprintln!("演示3: 不同闪烁模式");

  let patterns = [
    (BlinkPattern::Fast, "快速闪烁"),
    (BlinkPattern::Slow, "慢速闪烁"),
    (BlinkPattern::Heartbeat, "心跳模式"),
    (BlinkPattern::SOS, "SOS求救信号"),
  ];

  for (pattern, name) in patterns.iter() {
    rprintln!("  当前模式: {}", name);
    blinker.set_pattern(*pattern);

    // 闪烁2次完整周期
    match blinker.blink_times(2) {
      Ok(_) => rprintln!("  {} 演示完成", name),
      Err(_) => rprintln!("  {} 演示出错", name),
    }

    // 模式间暂停
    cortex_m::asm::delay(16_000_000);
  }

  // 演示4: 自定义闪烁模式
  rprintln!("演示4: 自定义闪烁模式");
  let custom_pattern = BlinkPattern::Custom(&[200, 100, 200, 100, 200, 800]);
  blinker.set_pattern(custom_pattern);

  match blinker.blink_times(3) {
    Ok(_) => rprintln!("  自定义模式演示完成"),
    Err(_) => rprintln!("  自定义模式演示出错"),
  }

  // 演示5: LED状态指示器
  rprintln!("演示5: LED状态指示器");
  let (led_controller, _delay) = blinker.release();
  let mut indicator = LedIndicator::new(led_controller);

  let states = [
    (IndicatorState::Ready, "就绪状态"),
    (IndicatorState::Busy, "忙碌状态"),
    (IndicatorState::Warning, "警告状态"),
    (IndicatorState::Error, "错误状态"),
    (IndicatorState::Off, "关闭状态"),
  ];

  for (state, name) in states.iter() {
    rprintln!("  设置状态: {}", name);
    indicator.set_state(*state).unwrap();

    // 对于需要闪烁的状态，手动更新几次
    if matches!(
      state,
      IndicatorState::Busy | IndicatorState::Warning | IndicatorState::Error
    ) {
      for _ in 0..5 {
        // 注意: 这里需要一个DelayNs实现，简化处理
        cortex_m::asm::delay(8_000_000);
      }
    } else {
      cortex_m::asm::delay(16_000_000);
    }
  }

  rprintln!("所有LED控制演示完成！");
  rprintln!("进入无限循环...");

  // 最终的无限闪烁循环
  let mut final_led = indicator.release();

  let mut counter = 0u32;
  loop {
    final_led.toggle().unwrap();
    counter += 1;

    if counter % 100 == 0 {
      rprintln!("系统运行正常，循环计数: {}", counter);
    }

    cortex_m::asm::delay(8_000_000);
  }
}

// 使用 panic-halt 库处理 panic，无需自定义 panic 处理函数
