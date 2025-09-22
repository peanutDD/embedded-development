#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use nrf52840_hal::{
    gpio::Level,
    pac::{CorePeripherals, Peripherals},
    prelude::*,
    timer::Timer,
};

use nrf52_platform::{
    BluetoothManager, GpioManager, SensorManager, SystemInit, Utils,
    debug::print_info,
    interrupts::{BUTTON_PRESSED, LED_STATE},
};

#[entry]
fn main() -> ! {
    // 系统初始化
    let core = CorePeripherals::take().unwrap();
    let device = Peripherals::take().unwrap();
    
    print_info("nRF52 蓝牙信标示例启动");
    
    // 初始化系统时钟
    SystemInit::init_clocks().unwrap();
    SystemInit::init_power_management().unwrap();
    SystemInit::init_rtc().unwrap();
    
    // 初始化GPIO
    let p0 = device.P0.split();
    let p1 = device.P1.split();
    let mut gpio = GpioManager::new(p0, p1);
    
    // 初始化蓝牙
    let mut bluetooth = BluetoothManager::new();
    bluetooth.init().unwrap();
    
    // 初始化传感器
    let mut sensors = SensorManager::new();
    
    // 初始化定时器
    let mut timer = Timer::new(device.TIMER0);
    
    print_info("系统初始化完成");
    
    // 启动蓝牙广播
    bluetooth.start_advertising().unwrap();
    gpio.set_led(1, true).unwrap(); // LED1表示蓝牙已启动
    
    let mut beacon_controller = BeaconController::new();
    let mut sensor_data_manager = SensorDataManager::new();
    
    print_info("蓝牙信标已启动");
    
    // 主循环
    loop {
        // 检查按钮状态
        if gpio.read_button(1).unwrap() {
            beacon_controller.toggle_beacon_mode();
            Utils::delay_ms(200); // 防抖
        }
        
        if gpio.read_button(2).unwrap() {
            sensor_data_manager.update_sensor_data(&mut sensors);
            Utils::delay_ms(200);
        }
        
        // 更新信标数据
        beacon_controller.update_beacon_data(&mut bluetooth, &sensor_data_manager);
        
        // LED状态指示
        beacon_controller.update_led_status(&mut gpio);
        
        // 处理蓝牙连接
        if bluetooth.is_connected() {
            gpio.set_led(2, true).unwrap(); // LED2表示已连接
            
            // 发送传感器数据
            let data = sensor_data_manager.get_formatted_data();
            bluetooth.send_data(&data).unwrap_or_else(|_| {
                print_info("数据发送失败");
            });
        } else {
            gpio.set_led(2, false).unwrap();
        }
        
        // 定期更新传感器数据
        sensor_data_manager.periodic_update(&mut sensors);
        
        Utils::delay_ms(100);
    }
}

// 信标控制器
struct BeaconController {
    beacon_mode: BeaconMode,
    last_update: u32,
    beacon_interval: u32,
}

#[derive(Clone, Copy)]
enum BeaconMode {
    Advertising,    // 广播模式
    Connectable,    // 可连接模式
    Scanning,       // 扫描模式
    Idle,          // 空闲模式
}

impl BeaconController {
    fn new() -> Self {
        Self {
            beacon_mode: BeaconMode::Advertising,
            last_update: 0,
            beacon_interval: 1000, // 1秒间隔
        }
    }
    
    fn toggle_beacon_mode(&mut self) {
        self.beacon_mode = match self.beacon_mode {
            BeaconMode::Advertising => BeaconMode::Connectable,
            BeaconMode::Connectable => BeaconMode::Scanning,
            BeaconMode::Scanning => BeaconMode::Idle,
            BeaconMode::Idle => BeaconMode::Advertising,
        };
        
        print_info("信标模式已切换");
    }
    
    fn update_beacon_data(&mut self, bluetooth: &mut BluetoothManager, sensor_data: &SensorDataManager) {
        match self.beacon_mode {
            BeaconMode::Advertising => {
                // 更新广播数据
                self.update_advertising_data(bluetooth, sensor_data);
            }
            BeaconMode::Connectable => {
                // 设置为可连接模式
                bluetooth.start_advertising().unwrap_or_else(|_| {
                    print_info("启动广播失败");
                });
            }
            BeaconMode::Scanning => {
                // 扫描其他设备
                self.scan_devices(bluetooth);
            }
            BeaconMode::Idle => {
                // 空闲模式，停止广播
                bluetooth.stop_advertising().unwrap_or_else(|_| {
                    print_info("停止广播失败");
                });
            }
        }
    }
    
    fn update_advertising_data(&self, bluetooth: &BluetoothManager, sensor_data: &SensorDataManager) {
        // 构建广播数据包
        let mut adv_data = [0u8; 31]; // BLE广播数据最大31字节
        
        // 设备名称
        adv_data[0] = 0x09; // 长度
        adv_data[1] = 0x09; // 完整本地名称
        adv_data[2..10].copy_from_slice(b"nRF52-BT");
        
        // 传感器数据
        let temp = sensor_data.get_temperature() as u16;
        adv_data[10] = 0x05; // 长度
        adv_data[11] = 0xFF; // 厂商特定数据
        adv_data[12] = 0x59; // 厂商ID低字节
        adv_data[13] = 0x00; // 厂商ID高字节
        adv_data[14] = (temp & 0xFF) as u8;
        adv_data[15] = ((temp >> 8) & 0xFF) as u8;
        
        // 这里应该调用实际的蓝牙API更新广播数据
        print_info("广播数据已更新");
    }
    
    fn scan_devices(&self, bluetooth: &BluetoothManager) {
        // 扫描附近的蓝牙设备
        print_info("正在扫描蓝牙设备...");
    }
    
    fn update_led_status(&self, gpio: &mut GpioManager) {
        match self.beacon_mode {
            BeaconMode::Advertising => {
                // LED3闪烁表示广播中
                gpio.toggle_led(3).unwrap();
            }
            BeaconMode::Connectable => {
                // LED3常亮表示可连接
                gpio.set_led(3, true).unwrap();
            }
            BeaconMode::Scanning => {
                // LED3快速闪烁表示扫描中
                gpio.toggle_led(3).unwrap();
                Utils::delay_ms(50);
                gpio.toggle_led(3).unwrap();
            }
            BeaconMode::Idle => {
                // LED3关闭表示空闲
                gpio.set_led(3, false).unwrap();
            }
        }
    }
}

// 传感器数据管理器
struct SensorDataManager {
    temperature: f32,
    humidity: f32,
    pressure: f32,
    last_update: u32,
    update_interval: u32,
}

impl SensorDataManager {
    fn new() -> Self {
        Self {
            temperature: 0.0,
            humidity: 0.0,
            pressure: 0.0,
            last_update: 0,
            update_interval: 5000, // 5秒更新间隔
        }
    }
    
    fn update_sensor_data(&mut self, sensors: &mut SensorManager) {
        match sensors.read_all_sensors() {
            Ok((temp, hum, press)) => {
                self.temperature = temp;
                self.humidity = hum;
                self.pressure = press;
                print_info("传感器数据已更新");
            }
            Err(_) => {
                print_info("传感器读取失败");
            }
        }
    }
    
    fn periodic_update(&mut self, sensors: &mut SensorManager) {
        // 定期自动更新传感器数据
        self.update_sensor_data(sensors);
    }
    
    fn get_temperature(&self) -> f32 {
        self.temperature
    }
    
    fn get_humidity(&self) -> f32 {
        self.humidity
    }
    
    fn get_pressure(&self) -> f32 {
        self.pressure
    }
    
    fn get_formatted_data(&self) -> [u8; 12] {
        let mut data = [0u8; 12];
        
        // 温度数据 (2字节)
        let temp_int = (self.temperature * 100.0) as u16;
        data[0] = (temp_int & 0xFF) as u8;
        data[1] = ((temp_int >> 8) & 0xFF) as u8;
        
        // 湿度数据 (2字节)
        let hum_int = (self.humidity * 100.0) as u16;
        data[2] = (hum_int & 0xFF) as u8;
        data[3] = ((hum_int >> 8) & 0xFF) as u8;
        
        // 气压数据 (4字节)
        let press_int = (self.pressure * 100.0) as u32;
        data[4] = (press_int & 0xFF) as u8;
        data[5] = ((press_int >> 8) & 0xFF) as u8;
        data[6] = ((press_int >> 16) & 0xFF) as u8;
        data[7] = ((press_int >> 24) & 0xFF) as u8;
        
        // 时间戳 (4字节)
        let timestamp = Utils::get_device_id() as u32; // 简化的时间戳
        data[8] = (timestamp & 0xFF) as u8;
        data[9] = ((timestamp >> 8) & 0xFF) as u8;
        data[10] = ((timestamp >> 16) & 0xFF) as u8;
        data[11] = ((timestamp >> 24) & 0xFF) as u8;
        
        data
    }
}

// 网络工具模块
mod network_utils {
    use super::*;
    
    pub fn format_mac_address(mac: &[u8; 6]) -> [u8; 17] {
        let mut formatted = [0u8; 17];
        for i in 0..6 {
            let hex_chars = format_hex_byte(mac[i]);
            formatted[i * 3] = hex_chars[0];
            formatted[i * 3 + 1] = hex_chars[1];
            if i < 5 {
                formatted[i * 3 + 2] = b':';
            }
        }
        formatted
    }
    
    fn format_hex_byte(byte: u8) -> [u8; 2] {
        let hex_chars = b"0123456789ABCDEF";
        [
            hex_chars[(byte >> 4) as usize],
            hex_chars[(byte & 0x0F) as usize],
        ]
    }
    
    pub fn calculate_rssi_distance(rssi: i8) -> f32 {
        // 根据RSSI估算距离（简化公式）
        let tx_power = -59.0; // 1米处的RSSI值
        if rssi == 0 {
            return -1.0;
        }
        
        let ratio = (tx_power - rssi as f32) / 20.0;
        10.0_f32.powf(ratio)
    }
}