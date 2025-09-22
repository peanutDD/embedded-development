use std::sync::{Arc, Mutex};
use std::time::Duration;

// 重新导出常用类型和特征
pub use esp_idf_hal as hal;
pub use esp_idf_svc as svc;
pub use esp_idf_sys as sys;
pub use embedded_hal;
pub use embedded_svc;

// GPIO相关
pub mod gpio {
    use crate::hal::{
        gpio::{Gpio2, Gpio4, Gpio5, Input, Output, PinDriver, PullDown, PullUp, PushPull},
        prelude::*,
    };
    
    // 常用引脚类型定义
    pub type LedPin = PinDriver<'static, Gpio2, Output>;
    pub type ButtonPin = PinDriver<'static, Gpio4, Input>;
    pub type SensorPowerPin = PinDriver<'static, Gpio5, Output>;
    
    // GPIO控制器
    pub struct GpioController {
        led: Option<LedPin>,
        button: Option<ButtonPin>,
        sensor_power: Option<SensorPowerPin>,
    }
    
    impl GpioController {
        pub fn new() -> Self {
            Self {
                led: None,
                button: None,
                sensor_power: None,
            }
        }
        
        pub fn init_led(&mut self, pin: Gpio2) -> Result<(), esp_idf_hal::gpio::EspError> {
            let led = PinDriver::output(pin)?;
            self.led = Some(led);
            Ok(())
        }
        
        pub fn init_button(&mut self, pin: Gpio4) -> Result<(), esp_idf_hal::gpio::EspError> {
            let button = PinDriver::input(pin)?;
            self.button = Some(button);
            Ok(())
        }
        
        pub fn led_on(&mut self) -> Result<(), esp_idf_hal::gpio::EspError> {
            if let Some(ref mut led) = self.led {
                led.set_high()
            } else {
                Err(esp_idf_hal::gpio::EspError::from_infallible::<std::convert::Infallible>(Ok(())))
            }
        }
        
        pub fn led_off(&mut self) -> Result<(), esp_idf_hal::gpio::EspError> {
            if let Some(ref mut led) = self.led {
                led.set_low()
            } else {
                Err(esp_idf_hal::gpio::EspError::from_infallible::<std::convert::Infallible>(Ok(())))
            }
        }
        
        pub fn is_button_pressed(&self) -> bool {
            if let Some(ref button) = self.button {
                button.is_high()
            } else {
                false
            }
        }
    }
}

// WiFi网络管理
pub mod wifi {
    use crate::svc::{
        eventloop::EspSystemEventLoop,
        hal::prelude::Peripherals,
        nvs::EspDefaultNvsPartition,
        wifi::{AuthMethod, ClientConfiguration, Configuration, EspWifi},
    };
    use embedded_svc::wifi::{ClientConfiguration as EmbeddedClientConfig, Wifi};
    use esp_idf_svc::wifi::WifiDriver;
    use std::sync::Arc;
    
    pub struct WifiManager {
        wifi: EspWifi<'static>,
        ssid: String,
        password: String,
    }
    
    impl WifiManager {
        pub fn new(
            ssid: &str,
            password: &str,
            peripherals: Peripherals,
            sys_loop: EspSystemEventLoop,
            nvs: EspDefaultNvsPartition,
        ) -> Result<Self, Box<dyn std::error::Error>> {
            let wifi_driver = WifiDriver::new(peripherals.modem, sys_loop.clone(), Some(nvs))?;
            let wifi = EspWifi::wrap(wifi_driver)?;
            
            Ok(Self {
                wifi,
                ssid: ssid.to_string(),
                password: password.to_string(),
            })
        }
        
        pub fn connect(&mut self) -> Result<(), Box<dyn std::error::Error>> {
            let wifi_config = Configuration::Client(ClientConfiguration {
                ssid: self.ssid.as_str().try_into()?,
                bssid: None,
                auth_method: AuthMethod::WPA2Personal,
                password: self.password.as_str().try_into()?,
                channel: None,
            });
            
            self.wifi.set_configuration(&wifi_config)?;
            self.wifi.start()?;
            
            log::info!("正在连接到WiFi网络: {}", self.ssid);
            self.wifi.connect()?;
            
            // 等待连接
            while !self.wifi.is_connected()? {
                std::thread::sleep(Duration::from_millis(100));
            }
            
            log::info!("WiFi连接成功!");
            Ok(())
        }
        
        pub fn disconnect(&mut self) -> Result<(), Box<dyn std::error::Error>> {
            self.wifi.disconnect()?;
            log::info!("WiFi已断开连接");
            Ok(())
        }
        
        pub fn is_connected(&self) -> Result<bool, Box<dyn std::error::Error>> {
            Ok(self.wifi.is_connected()?)
        }
        
        pub fn get_ip_info(&self) -> Result<embedded_svc::ipv4::IpInfo, Box<dyn std::error::Error>> {
            Ok(self.wifi.sta_netif().get_ip_info()?)
        }
    }
}

// HTTP服务器
pub mod http {
    use crate::svc::{
        http::server::{Configuration, EspHttpServer},
        io::Write,
    };
    use embedded_svc::{
        http::{Headers, Method},
        io::Read,
    };
    use std::sync::{Arc, Mutex};
    
    pub struct HttpServer {
        server: EspHttpServer<'static>,
        data: Arc<Mutex<ServerData>>,
    }
    
    #[derive(Default)]
    pub struct ServerData {
        pub counter: u32,
        pub sensor_data: f32,
        pub status: String,
    }
    
    impl HttpServer {
        pub fn new() -> Result<Self, Box<dyn std::error::Error>> {
            let server_config = Configuration::default();
            let mut server = EspHttpServer::new(&server_config)?;
            let data = Arc::new(Mutex::new(ServerData::default()));
            
            // 设置路由
            let data_clone = data.clone();
            server.fn_handler("/", Method::Get, move |request| {
                let data = data_clone.lock().unwrap();
                let html = format!(
                    r#"
                    <!DOCTYPE html>
                    <html>
                    <head>
                        <title>ESP32 Web Server</title>
                        <meta charset="UTF-8">
                    </head>
                    <body>
                        <h1>ESP32 Web服务器</h1>
                        <p>计数器: {}</p>
                        <p>传感器数据: {:.2}</p>
                        <p>状态: {}</p>
                        <p><a href="/api/data">API数据</a></p>
                    </body>
                    </html>
                    "#,
                    data.counter, data.sensor_data, data.status
                );
                
                let mut response = request.into_ok_response()?;
                response.write_all(html.as_bytes())?;
                Ok(())
            })?;
            
            let data_clone = data.clone();
            server.fn_handler("/api/data", Method::Get, move |request| {
                let data = data_clone.lock().unwrap();
                let json = serde_json::json!({
                    "counter": data.counter,
                    "sensor_data": data.sensor_data,
                    "status": data.status,
                    "timestamp": chrono::Utc::now().timestamp()
                });
                
                let mut response = request.into_ok_response()?;
                response.write_all(json.to_string().as_bytes())?;
                Ok(())
            })?;
            
            Ok(Self { server, data })
        }
        
        pub fn update_data(&self, counter: u32, sensor_data: f32, status: &str) {
            let mut data = self.data.lock().unwrap();
            data.counter = counter;
            data.sensor_data = sensor_data;
            data.status = status.to_string();
        }
    }
}

// 传感器管理
pub mod sensors {
    use crate::hal::{
        adc::{AdcChannelDriver, AdcDriver, Atten11dB, ADC1},
        gpio::Gpio36,
        i2c::{I2cConfig, I2cDriver},
        prelude::*,
    };
    use std::sync::{Arc, Mutex};
    
    pub struct SensorManager {
        adc: Option<AdcDriver<'static, ADC1>>,
        adc_channel: Option<AdcChannelDriver<'static, Gpio36, Atten11dB<ADC1>>>,
        i2c: Option<I2cDriver<'static>>,
        data: Arc<Mutex<SensorData>>,
    }
    
    #[derive(Default, Clone)]
    pub struct SensorData {
        pub temperature: f32,
        pub humidity: f32,
        pub pressure: f32,
        pub light_level: u16,
        pub battery_voltage: f32,
    }
    
    impl SensorManager {
        pub fn new() -> Self {
            Self {
                adc: None,
                adc_channel: None,
                i2c: None,
                data: Arc::new(Mutex::new(SensorData::default())),
            }
        }
        
        pub fn init_adc(&mut self, adc1: esp_idf_hal::adc::ADC1, pin: Gpio36) -> Result<(), Box<dyn std::error::Error>> {
            let adc_config = esp_idf_hal::adc::config::Config::new().calibration(true);
            let adc = AdcDriver::new(adc1, &adc_config)?;
            let adc_channel = AdcChannelDriver::new(pin)?;
            
            self.adc = Some(adc);
            self.adc_channel = Some(adc_channel);
            Ok(())
        }
        
        pub fn read_adc(&mut self) -> Result<u16, Box<dyn std::error::Error>> {
            if let (Some(ref mut adc), Some(ref mut channel)) = (&mut self.adc, &mut self.adc_channel) {
                Ok(adc.read(channel)?)
            } else {
                Err("ADC未初始化".into())
            }
        }
        
        pub fn update_sensor_data(&self) {
            // 模拟传感器数据更新
            let mut data = self.data.lock().unwrap();
            data.temperature = 25.0 + (rand::random::<f32>() - 0.5) * 10.0;
            data.humidity = 50.0 + (rand::random::<f32>() - 0.5) * 20.0;
            data.pressure = 1013.25 + (rand::random::<f32>() - 0.5) * 50.0;
            data.light_level = (rand::random::<f32>() * 1024.0) as u16;
            data.battery_voltage = 3.3 + (rand::random::<f32>() - 0.5) * 0.6;
        }
        
        pub fn get_data(&self) -> SensorData {
            self.data.lock().unwrap().clone()
        }
    }
}

// 系统管理
pub mod system {
    use crate::svc::{
        eventloop::EspSystemEventLoop,
        nvs::EspDefaultNvsPartition,
        hal::prelude::Peripherals,
    };
    
    pub struct SystemManager {
        pub peripherals: Peripherals,
        pub sys_loop: EspSystemEventLoop,
        pub nvs: EspDefaultNvsPartition,
    }
    
    impl SystemManager {
        pub fn new() -> Result<Self, Box<dyn std::error::Error>> {
            esp_idf_svc::sys::link_patches();
            esp_idf_svc::log::EspLogger::initialize_default();
            
            let peripherals = Peripherals::take()?;
            let sys_loop = EspSystemEventLoop::take()?;
            let nvs = EspDefaultNvsPartition::take()?;
            
            Ok(Self {
                peripherals,
                sys_loop,
                nvs,
            })
        }
        
        pub fn get_chip_info(&self) -> String {
            format!(
                "ESP32 - 芯片型号: {}, 版本: {}",
                esp_idf_svc::sys::esp_get_idf_version(),
                "ESP32"
            )
        }
        
        pub fn get_free_heap(&self) -> u32 {
            unsafe { esp_idf_svc::sys::esp_get_free_heap_size() }
        }
        
        pub fn restart(&self) {
            log::info!("系统重启中...");
            unsafe {
                esp_idf_svc::sys::esp_restart();
            }
        }
    }
}

// 错误处理
#[derive(Debug)]
pub enum PlatformError {
    InitializationFailed(String),
    NetworkError(String),
    SensorError(String),
    ConfigurationError(String),
}

impl std::fmt::Display for PlatformError {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            PlatformError::InitializationFailed(msg) => write!(f, "初始化失败: {}", msg),
            PlatformError::NetworkError(msg) => write!(f, "网络错误: {}", msg),
            PlatformError::SensorError(msg) => write!(f, "传感器错误: {}", msg),
            PlatformError::ConfigurationError(msg) => write!(f, "配置错误: {}", msg),
        }
    }
}

impl std::error::Error for PlatformError {}

// 实用工具
pub mod utils {
    use std::time::Duration;
    
    pub fn delay_ms(ms: u64) {
        std::thread::sleep(Duration::from_millis(ms));
    }
    
    pub fn delay_us(us: u64) {
        std::thread::sleep(Duration::from_micros(us));
    }
    
    pub fn get_timestamp() -> u64 {
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_secs()
    }
}