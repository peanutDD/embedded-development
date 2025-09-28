use esp32_examples::{
  gpio::GpioController,
  http::HttpServer,
  sensors::{SensorData, SensorManager},
  system::SystemManager,
  utils,
  wifi::WifiManager,
};
use log::*;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

// WiFi配置 - 请修改为您的网络信息
const WIFI_SSID: &str = "Your_WiFi_SSID";
const WIFI_PASSWORD: &str = "Your_WiFi_Password";

fn main() -> Result<(), Box<dyn std::error::Error>> {
  info!("ESP32 WiFi连接示例启动");

  // 系统初始化
  let mut system = SystemManager::new()?;
  info!("{}", system.get_chip_info());
  info!("可用堆内存: {} bytes", system.get_free_heap());

  // GPIO初始化
  let mut gpio_controller = GpioController::new();
  gpio_controller.init_led(system.peripherals.pins.gpio2)?;
  gpio_controller.init_button(system.peripherals.pins.gpio4)?;

  // 传感器初始化
  let mut sensor_manager = SensorManager::new();
  sensor_manager.init_adc(system.peripherals.adc1, system.peripherals.pins.gpio36)?;

  // WiFi连接
  let mut wifi_manager = WifiManager::new(
    WIFI_SSID,
    WIFI_PASSWORD,
    system.peripherals,
    system.sys_loop,
    system.nvs,
  )?;

  info!("正在连接WiFi...");
  gpio_controller.led_on()?; // LED指示连接中

  match wifi_manager.connect() {
    Ok(_) => {
      info!("WiFi连接成功!");
      let ip_info = wifi_manager.get_ip_info()?;
      info!("IP地址: {}", ip_info.ip);
      info!("网关: {}", ip_info.gw);
      info!("子网掩码: {}", ip_info.netmask);

      // LED快闪表示连接成功
      for _ in 0..5 {
        gpio_controller.led_off()?;
        utils::delay_ms(100);
        gpio_controller.led_on()?;
        utils::delay_ms(100);
      }
    }
    Err(e) => {
      error!("WiFi连接失败: {}", e);
      // LED慢闪表示连接失败
      for _ in 0..10 {
        gpio_controller.led_off()?;
        utils::delay_ms(500);
        gpio_controller.led_on()?;
        utils::delay_ms(500);
      }
      return Err(e);
    }
  }

  // 启动HTTP服务器
  let http_server = Arc::new(Mutex::new(HttpServer::new()?));
  info!(
    "HTTP服务器已启动，访问: http://{}",
    wifi_manager.get_ip_info()?.ip
  );

  // 数据共享
  let sensor_data = Arc::new(Mutex::new(SensorData::default()));
  let counter = Arc::new(Mutex::new(0u32));

  // 传感器数据更新线程
  let sensor_data_clone = sensor_data.clone();
  let sensor_thread = thread::spawn(move || {
    let mut local_sensor_manager = SensorManager::new();
    loop {
      local_sensor_manager.update_sensor_data();
      let data = local_sensor_manager.get_data();

      {
        let mut shared_data = sensor_data_clone.lock().unwrap();
        *shared_data = data;
      }

      info!(
        "传感器数据更新: 温度={:.1}°C, 湿度={:.1}%, 气压={:.1}hPa",
        data.temperature, data.humidity, data.pressure
      );

      thread::sleep(Duration::from_secs(5));
    }
  });

  // 网络状态监控线程
  let counter_clone = counter.clone();
  let network_thread = thread::spawn(move || loop {
    match wifi_manager.is_connected() {
      Ok(true) => {
        info!("网络连接正常");
        let mut count = counter_clone.lock().unwrap();
        *count += 1;
      }
      Ok(false) => {
        warn!("网络连接断开，尝试重连...");
        if let Err(e) = wifi_manager.connect() {
          error!("重连失败: {}", e);
        }
      }
      Err(e) => {
        error!("网络状态检查失败: {}", e);
      }
    }

    thread::sleep(Duration::from_secs(10));
  });

  // 主循环
  let mut loop_counter = 0u32;
  loop {
    // 更新HTTP服务器数据
    {
      let sensor_data_guard = sensor_data.lock().unwrap();
      let counter_guard = counter.lock().unwrap();
      let server_guard = http_server.lock().unwrap();

      let status = if wifi_manager.is_connected().unwrap_or(false) {
        "在线"
      } else {
        "离线"
      };

      server_guard.update_data(*counter_guard, sensor_data_guard.temperature, status);
    }

    // 检查按钮状态
    if gpio_controller.is_button_pressed() {
      info!("按钮被按下 - 执行特殊操作");

      // 显示系统信息
      info!("系统运行时间: {} 秒", loop_counter);
      info!("可用堆内存: {} bytes", system.get_free_heap());

      // LED指示
      for _ in 0..3 {
        gpio_controller.led_off()?;
        utils::delay_ms(200);
        gpio_controller.led_on()?;
        utils::delay_ms(200);
      }

      utils::delay_ms(1000); // 防抖
    }

    // LED心跳指示
    if loop_counter % 20 == 0 {
      gpio_controller.led_off()?;
      utils::delay_ms(50);
      gpio_controller.led_on()?;
    }

    loop_counter += 1;
    utils::delay_ms(100);

    // 每分钟输出一次状态
    if loop_counter % 600 == 0 {
      info!("系统运行正常 - 循环计数: {}", loop_counter);
      info!(
        "网络状态: {}",
        if wifi_manager.is_connected().unwrap_or(false) {
          "已连接"
        } else {
          "未连接"
        }
      );

      let sensor_data_guard = sensor_data.lock().unwrap();
      info!(
        "当前传感器数据: 温度={:.1}°C, 湿度={:.1}%, 光照={}",
        sensor_data_guard.temperature, sensor_data_guard.humidity, sensor_data_guard.light_level
      );
    }
  }
}

// WiFi事件处理
struct WiFiEventHandler {
  led_controller: Arc<Mutex<GpioController>>,
}

impl WiFiEventHandler {
  pub fn new(led_controller: Arc<Mutex<GpioController>>) -> Self {
    Self { led_controller }
  }

  pub fn handle_connected(&self) {
    info!("WiFi连接事件: 已连接");
    if let Ok(mut controller) = self.led_controller.lock() {
      // 快速闪烁表示连接成功
      for _ in 0..3 {
        let _ = controller.led_off();
        utils::delay_ms(100);
        let _ = controller.led_on();
        utils::delay_ms(100);
      }
    }
  }

  pub fn handle_disconnected(&self) {
    warn!("WiFi连接事件: 已断开");
    if let Ok(mut controller) = self.led_controller.lock() {
      // 慢速闪烁表示断开连接
      for _ in 0..5 {
        let _ = controller.led_off();
        utils::delay_ms(300);
        let _ = controller.led_on();
        utils::delay_ms(300);
      }
    }
  }

  pub fn handle_got_ip(&self, ip: &str) {
    info!("WiFi连接事件: 获得IP地址 {}", ip);
    if let Ok(mut controller) = self.led_controller.lock() {
      // 长亮表示获得IP
      let _ = controller.led_on();
    }
  }
}

// 网络工具函数
mod network_utils {
  use log::*;
  use std::time::Duration;

  pub fn ping_test(host: &str) -> Result<Duration, Box<dyn std::error::Error>> {
    info!("正在ping {}", host);

    // 这里应该实现实际的ping功能
    // ESP32可以使用esp_ping库
    let start = std::time::Instant::now();

    // 模拟ping延迟
    std::thread::sleep(Duration::from_millis(50));

    let duration = start.elapsed();
    info!("Ping {} 成功，延迟: {:?}", host, duration);

    Ok(duration)
  }

  pub fn scan_networks() -> Result<Vec<String>, Box<dyn std::error::Error>> {
    info!("扫描可用WiFi网络...");

    // 这里应该实现实际的网络扫描
    // 返回模拟数据
    let networks = vec![
      "Network_1".to_string(),
      "Network_2".to_string(),
      "Network_3".to_string(),
    ];

    info!("发现 {} 个网络", networks.len());
    for (i, network) in networks.iter().enumerate() {
      info!("  {}: {}", i + 1, network);
    }

    Ok(networks)
  }

  pub fn get_signal_strength() -> i8 {
    // 返回信号强度 (dBm)
    -45 // 模拟强信号
  }
}
