#![no_std]
#![no_main]

use cortex_m_rt::entry;
use stm32f4xx_hal::{
    gpio::{
        gpioa::{PA5, PA6, PA7},
        gpiob::{PB0, PB6, PB7, PB8, PB9},
        gpioc::PC13,
        Alternate, AlternateOD, Output, PushPull,
    },
    i2c::{I2c, Mode as I2cMode},
    pac,
    prelude::*,
    spi::{Mode as SpiMode, Phase, Polarity, Spi},
    timer::{Timer, Event},
};
use panic_halt as _;
use heapless::{Vec, String};

mod sensors;
mod flash_storage;
mod data_logger;

use sensors::{SensorData, SensorManager};
use flash_storage::FlashStorage;
use data_logger::DataLogger;

// 系统配置
const SAMPLE_INTERVAL_MS: u32 = 1000; // 1秒采样间隔
const LOG_BUFFER_SIZE: usize = 100;   // 日志缓冲区大小
const FLASH_LOG_INTERVAL: u32 = 10;   // 每10次采样写入Flash

// 类型别名
type I2cType = I2c<pac::I2C1, (PB6<AlternateOD<4>>, PB7<AlternateOD<4>>)>;
type SpiType = Spi<pac::SPI1, (PA5<Alternate<5>>, PA6<Alternate<5>>, PA7<Alternate<5>>)>;
type CsPin = PB0<Output<PushPull>>;
type LedPin = PC13<Output<PushPull>>;
type TimerType = Timer<pac::TIM2>;

#[entry]
fn main() -> ! {
    // 获取外设句柄
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();
    
    // 配置时钟
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(84.MHz()).freeze();
    
    // 配置 GPIO
    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();
    
    // 配置状态 LED
    let mut led = gpioc.pc13.into_push_pull_output();
    led.set_high();
    
    // 配置 I2C 引脚 (传感器通信)
    let scl = gpiob.pb6.into_alternate_open_drain();
    let sda = gpiob.pb7.into_alternate_open_drain();
    
    // 配置 SPI 引脚 (Flash 存储)
    let sck = gpioa.pa5.into_alternate();
    let miso = gpioa.pa6.into_alternate();
    let mosi = gpioa.pa7.into_alternate();
    let mut cs = gpiob.pb0.into_push_pull_output();
    cs.set_high();
    
    // 配置额外的控制引脚
    let mut sensor_power = gpiob.pb8.into_push_pull_output(); // 传感器电源控制
    let mut flash_power = gpiob.pb9.into_push_pull_output();  // Flash 电源控制
    
    // 初始化外设
    let i2c = I2c::new(
        dp.I2C1,
        (scl, sda),
        I2cMode::Standard { frequency: 100.kHz() },
        &clocks,
    );
    
    let spi_mode = SpiMode {
        polarity: Polarity::IdleLow,
        phase: Phase::CaptureOnFirstTransition,
    };
    
    let spi = Spi::new(
        dp.SPI1,
        (sck, miso, mosi),
        spi_mode,
        8.MHz(),
        &clocks,
    );
    
    // 配置定时器
    let mut timer = Timer::new(dp.TIM2, &clocks);
    timer.start(SAMPLE_INTERVAL_MS.millis());
    timer.listen(Event::TimeOut);
    
    // 系统启动指示
    startup_sequence(&mut led);
    
    // 初始化系统组件
    sensor_power.set_high(); // 打开传感器电源
    flash_power.set_high();  // 打开Flash电源
    delay_ms(100); // 等待电源稳定
    
    let mut sensor_manager = SensorManager::new(i2c);
    let mut flash_storage = FlashStorage::new(spi, cs);
    let mut data_logger = DataLogger::new();
    
    // 初始化传感器
    if sensor_manager.initialize().is_ok() {
        led_blink(&mut led, 2, 200); // 传感器初始化成功
    } else {
        led_blink(&mut led, 5, 100); // 传感器初始化失败
    }
    
    // 初始化Flash存储
    if flash_storage.initialize().is_ok() {
        led_blink(&mut led, 3, 200); // Flash初始化成功
    } else {
        led_blink(&mut led, 6, 100); // Flash初始化失败
    }
    
    let mut sample_count = 0u32;
    let mut error_count = 0u32;
    
    // 主循环
    loop {
        // 等待定时器中断
        if timer.wait().is_ok() {
            led.set_low(); // 采样指示
            
            // 读取传感器数据
            match sensor_manager.read_all_sensors() {
                Ok(sensor_data) => {
                    // 记录数据到缓冲区
                    data_logger.log_data(sensor_data, sample_count);
                    
                    // 定期写入Flash
                    if sample_count % FLASH_LOG_INTERVAL == 0 {
                        if let Err(_) = write_data_to_flash(&mut flash_storage, &mut data_logger) {
                            error_count += 1;
                        }
                    }
                    
                    sample_count += 1;
                }
                Err(_) => {
                    error_count += 1;
                    // 传感器读取失败指示
                    led_blink(&mut led, 1, 50);
                }
            }
            
            led.set_high(); // 采样完成
            
            // 状态指示
            if sample_count % 60 == 0 { // 每分钟显示一次状态
                display_system_status(&mut led, sample_count, error_count);
            }
            
            // 错误处理
            if error_count > 10 {
                // 尝试重新初始化系统
                system_recovery(&mut sensor_manager, &mut flash_storage, &mut led);
                error_count = 0;
            }
        }
        
        // 低功耗等待
        cortex_m::asm::wfi();
    }
}

fn startup_sequence(led: &mut LedPin) {
    // 系统启动指示序列
    for i in 1..=5 {
        led.set_low();
        delay_ms(i * 50);
        led.set_high();
        delay_ms(100);
    }
}

fn write_data_to_flash(flash_storage: &mut FlashStorage, data_logger: &mut DataLogger) -> Result<(), ()> {
    // 获取缓冲区中的数据
    let data_buffer = data_logger.get_buffer();
    
    if !data_buffer.is_empty() {
        // 写入Flash
        flash_storage.write_sensor_data(data_buffer)?;
        
        // 清空缓冲区
        data_logger.clear_buffer();
    }
    
    Ok(())
}

fn display_system_status(led: &mut LedPin, sample_count: u32, error_count: u32) {
    // 显示系统运行状态
    // 长亮表示系统正常运行
    led.set_low();
    delay_ms(1000);
    led.set_high();
    
    // 根据错误数量闪烁
    if error_count > 0 {
        let blink_count = if error_count > 10 { 10 } else { error_count };
        for _ in 0..blink_count {
            led.set_low();
            delay_ms(100);
            led.set_high();
            delay_ms(100);
        }
    }
    
    delay_ms(500);
    
    // 显示采样计数 (每1000次一个长闪烁)
    let thousands = sample_count / 1000;
    for _ in 0..thousands {
        led.set_low();
        delay_ms(300);
        led.set_high();
        delay_ms(300);
    }
}

fn system_recovery(sensor_manager: &mut SensorManager, flash_storage: &mut FlashStorage, led: &mut LedPin) {
    // 系统恢复程序
    led_blink(led, 10, 50); // 恢复指示
    
    // 重新初始化传感器
    if sensor_manager.initialize().is_ok() {
        led_blink(led, 2, 200);
    }
    
    // 重新初始化Flash
    if flash_storage.initialize().is_ok() {
        led_blink(led, 3, 200);
    }
    
    delay_ms(1000);
}

fn led_blink(led: &mut LedPin, count: u32, delay: u32) {
    for _ in 0..count {
        led.set_low();
        delay_ms(delay);
        led.set_high();
        delay_ms(delay);
    }
}

fn delay_ms(ms: u32) {
    for _ in 0..(ms * 21000) {
        cortex_m::asm::nop();
    }
}

// 传感器模块
mod sensors {
    use super::*;
    use heapless::Vec;
    
    #[derive(Debug, Clone)]
    pub struct SensorData {
        pub timestamp: u32,
        pub temperature: f32,
        pub pressure: f32,
        pub humidity: f32,
        pub accel_x: f32,
        pub accel_y: f32,
        pub accel_z: f32,
        pub gyro_x: f32,
        pub gyro_y: f32,
        pub gyro_z: f32,
    }
    
    impl Default for SensorData {
        fn default() -> Self {
            Self {
                timestamp: 0,
                temperature: 0.0,
                pressure: 0.0,
                humidity: 0.0,
                accel_x: 0.0,
                accel_y: 0.0,
                accel_z: 0.0,
                gyro_x: 0.0,
                gyro_y: 0.0,
                gyro_z: 0.0,
            }
        }
    }
    
    pub struct SensorManager {
        i2c: I2cType,
        bmp280_initialized: bool,
        mpu6050_initialized: bool,
    }
    
    impl SensorManager {
        pub fn new(i2c: I2cType) -> Self {
            Self {
                i2c,
                bmp280_initialized: false,
                mpu6050_initialized: false,
            }
        }
        
        pub fn initialize(&mut self) -> Result<(), ()> {
            // 初始化 BMP280 (温度/压力传感器)
            self.bmp280_initialized = self.init_bmp280().is_ok();
            
            // 初始化 MPU6050 (加速度计/陀螺仪)
            self.mpu6050_initialized = self.init_mpu6050().is_ok();
            
            if self.bmp280_initialized || self.mpu6050_initialized {
                Ok(())
            } else {
                Err(())
            }
        }
        
        pub fn read_all_sensors(&mut self) -> Result<SensorData, ()> {
            let mut data = SensorData::default();
            data.timestamp = get_system_time();
            
            // 读取 BMP280 数据
            if self.bmp280_initialized {
                if let Ok((temp, press)) = self.read_bmp280() {
                    data.temperature = temp;
                    data.pressure = press;
                }
            }
            
            // 读取 MPU6050 数据
            if self.mpu6050_initialized {
                if let Ok((accel, gyro)) = self.read_mpu6050() {
                    data.accel_x = accel.0;
                    data.accel_y = accel.1;
                    data.accel_z = accel.2;
                    data.gyro_x = gyro.0;
                    data.gyro_y = gyro.1;
                    data.gyro_z = gyro.2;
                }
            }
            
            Ok(data)
        }
        
        fn init_bmp280(&mut self) -> Result<(), ()> {
            const BMP280_ADDR: u8 = 0x76;
            const BMP280_ID_REG: u8 = 0xD0;
            const BMP280_CTRL_MEAS: u8 = 0xF4;
            const BMP280_CONFIG: u8 = 0xF5;
            
            // 读取芯片ID
            let mut id_buffer = [0u8; 1];
            self.i2c.write_read(BMP280_ADDR, &[BMP280_ID_REG], &mut id_buffer).map_err(|_| ())?;
            
            if id_buffer[0] != 0x58 { // BMP280 ID
                return Err(());
            }
            
            // 配置传感器
            self.i2c.write(BMP280_ADDR, &[BMP280_CTRL_MEAS, 0x27]).map_err(|_| ())?; // 正常模式
            self.i2c.write(BMP280_ADDR, &[BMP280_CONFIG, 0xA0]).map_err(|_| ())?;    // 配置
            
            delay_ms(10);
            Ok(())
        }
        
        fn read_bmp280(&mut self) -> Result<(f32, f32), ()> {
            const BMP280_ADDR: u8 = 0x76;
            const BMP280_PRESS_MSB: u8 = 0xF7;
            
            // 读取原始数据
            let mut data = [0u8; 6];
            self.i2c.write_read(BMP280_ADDR, &[BMP280_PRESS_MSB], &mut data).map_err(|_| ())?;
            
            // 解析压力数据
            let press_raw = ((data[0] as u32) << 12) | ((data[1] as u32) << 4) | ((data[2] as u32) >> 4);
            
            // 解析温度数据
            let temp_raw = ((data[3] as u32) << 12) | ((data[4] as u32) << 4) | ((data[5] as u32) >> 4);
            
            // 简化的温度和压力计算 (实际应用中需要使用校准参数)
            let temperature = (temp_raw as f32) / 5120.0 - 40.0;
            let pressure = (press_raw as f32) / 256.0;
            
            Ok((temperature, pressure))
        }
        
        fn init_mpu6050(&mut self) -> Result<(), ()> {
            const MPU6050_ADDR: u8 = 0x68;
            const MPU6050_WHO_AM_I: u8 = 0x75;
            const MPU6050_PWR_MGMT_1: u8 = 0x6B;
            const MPU6050_GYRO_CONFIG: u8 = 0x1B;
            const MPU6050_ACCEL_CONFIG: u8 = 0x1C;
            
            // 读取设备ID
            let mut id_buffer = [0u8; 1];
            self.i2c.write_read(MPU6050_ADDR, &[MPU6050_WHO_AM_I], &mut id_buffer).map_err(|_| ())?;
            
            if id_buffer[0] != 0x68 { // MPU6050 ID
                return Err(());
            }
            
            // 唤醒设备
            self.i2c.write(MPU6050_ADDR, &[MPU6050_PWR_MGMT_1, 0x00]).map_err(|_| ())?;
            delay_ms(10);
            
            // 配置陀螺仪 (±250°/s)
            self.i2c.write(MPU6050_ADDR, &[MPU6050_GYRO_CONFIG, 0x00]).map_err(|_| ())?;
            
            // 配置加速度计 (±2g)
            self.i2c.write(MPU6050_ADDR, &[MPU6050_ACCEL_CONFIG, 0x00]).map_err(|_| ())?;
            
            delay_ms(10);
            Ok(())
        }
        
        fn read_mpu6050(&mut self) -> Result<((f32, f32, f32), (f32, f32, f32)), ()> {
            const MPU6050_ADDR: u8 = 0x68;
            const MPU6050_ACCEL_XOUT_H: u8 = 0x3B;
            
            // 读取加速度计和陀螺仪数据 (14字节)
            let mut data = [0u8; 14];
            self.i2c.write_read(MPU6050_ADDR, &[MPU6050_ACCEL_XOUT_H], &mut data).map_err(|_| ())?;
            
            // 解析加速度数据
            let accel_x = i16::from_be_bytes([data[0], data[1]]) as f32 / 16384.0;
            let accel_y = i16::from_be_bytes([data[2], data[3]]) as f32 / 16384.0;
            let accel_z = i16::from_be_bytes([data[4], data[5]]) as f32 / 16384.0;
            
            // 跳过温度数据 (data[6], data[7])
            
            // 解析陀螺仪数据
            let gyro_x = i16::from_be_bytes([data[8], data[9]]) as f32 / 131.0;
            let gyro_y = i16::from_be_bytes([data[10], data[11]]) as f32 / 131.0;
            let gyro_z = i16::from_be_bytes([data[12], data[13]]) as f32 / 131.0;
            
            Ok(((accel_x, accel_y, accel_z), (gyro_x, gyro_y, gyro_z)))
        }
    }
    
    fn get_system_time() -> u32 {
        // 简化的时间戳 (实际应用中应使用RTC)
        static mut COUNTER: u32 = 0;
        unsafe {
            COUNTER += 1;
            COUNTER
        }
    }
}

// Flash存储模块
mod flash_storage {
    use super::*;
    use super::sensors::SensorData;
    use heapless::Vec;
    
    pub struct FlashStorage {
        spi: SpiType,
        cs: CsPin,
        current_address: u32,
        initialized: bool,
    }
    
    impl FlashStorage {
        pub fn new(spi: SpiType, cs: CsPin) -> Self {
            Self {
                spi,
                cs,
                current_address: 0x1000, // 从4KB开始存储数据
                initialized: false,
            }
        }
        
        pub fn initialize(&mut self) -> Result<(), ()> {
            // 检查Flash芯片
            let jedec_id = self.read_jedec_id();
            
            if jedec_id[0] == 0xEF && jedec_id[1] == 0x40 {
                self.initialized = true;
                Ok(())
            } else {
                Err(())
            }
        }
        
        pub fn write_sensor_data(&mut self, data: &[SensorData]) -> Result<(), ()> {
            if !self.initialized {
                return Err(());
            }
            
            for sensor_data in data {
                self.write_single_record(sensor_data)?;
            }
            
            Ok(())
        }
        
        fn write_single_record(&mut self, data: &SensorData) -> Result<(), ()> {
            // 将传感器数据序列化为字节数组
            let mut buffer = [0u8; 44]; // 11个f32值 * 4字节
            
            // 简化的序列化 (实际应用中可能需要更复杂的格式)
            let timestamp_bytes = data.timestamp.to_le_bytes();
            buffer[0..4].copy_from_slice(&timestamp_bytes);
            
            let temp_bytes = data.temperature.to_le_bytes();
            buffer[4..8].copy_from_slice(&temp_bytes);
            
            let press_bytes = data.pressure.to_le_bytes();
            buffer[8..12].copy_from_slice(&press_bytes);
            
            let humidity_bytes = data.humidity.to_le_bytes();
            buffer[12..16].copy_from_slice(&humidity_bytes);
            
            let accel_x_bytes = data.accel_x.to_le_bytes();
            buffer[16..20].copy_from_slice(&accel_x_bytes);
            
            let accel_y_bytes = data.accel_y.to_le_bytes();
            buffer[20..24].copy_from_slice(&accel_y_bytes);
            
            let accel_z_bytes = data.accel_z.to_le_bytes();
            buffer[24..28].copy_from_slice(&accel_z_bytes);
            
            let gyro_x_bytes = data.gyro_x.to_le_bytes();
            buffer[28..32].copy_from_slice(&gyro_x_bytes);
            
            let gyro_y_bytes = data.gyro_y.to_le_bytes();
            buffer[32..36].copy_from_slice(&gyro_y_bytes);
            
            let gyro_z_bytes = data.gyro_z.to_le_bytes();
            buffer[36..40].copy_from_slice(&gyro_z_bytes);
            
            // 写入校验和
            let checksum = self.calculate_checksum(&buffer[0..40]);
            let checksum_bytes = checksum.to_le_bytes();
            buffer[40..44].copy_from_slice(&checksum_bytes);
            
            // 检查是否需要擦除新扇区
            if self.current_address % 4096 == 0 {
                self.sector_erase(self.current_address)?;
            }
            
            // 写入数据
            self.page_program(self.current_address, &buffer)?;
            self.current_address += buffer.len() as u32;
            
            Ok(())
        }
        
        fn calculate_checksum(&self, data: &[u8]) -> u32 {
            let mut checksum = 0u32;
            for &byte in data {
                checksum = checksum.wrapping_add(byte as u32);
            }
            checksum
        }
        
        // Flash 基本操作
        fn read_jedec_id(&mut self) -> [u8; 3] {
            let mut id = [0u8; 3];
            
            self.cs.set_low();
            let _ = self.spi.send(0x9F); // JEDEC ID命令
            
            for i in 0..3 {
                id[i] = self.spi_transfer(0x00);
            }
            
            self.cs.set_high();
            id
        }
        
        fn sector_erase(&mut self, address: u32) -> Result<(), ()> {
            self.write_enable();
            
            self.cs.set_low();
            let _ = self.spi.send(0x20); // 扇区擦除命令
            let _ = self.spi.send((address >> 16) as u8);
            let _ = self.spi.send((address >> 8) as u8);
            let _ = self.spi.send(address as u8);
            self.cs.set_high();
            
            self.wait_busy();
            Ok(())
        }
        
        fn page_program(&mut self, address: u32, data: &[u8]) -> Result<(), ()> {
            self.write_enable();
            
            self.cs.set_low();
            let _ = self.spi.send(0x02); // 页编程命令
            let _ = self.spi.send((address >> 16) as u8);
            let _ = self.spi.send((address >> 8) as u8);
            let _ = self.spi.send(address as u8);
            
            for &byte in data {
                let _ = self.spi.send(byte);
            }
            
            self.cs.set_high();
            self.wait_busy();
            Ok(())
        }
        
        fn write_enable(&mut self) {
            self.cs.set_low();
            let _ = self.spi.send(0x06);
            self.cs.set_high();
        }
        
        fn wait_busy(&mut self) {
            loop {
                self.cs.set_low();
                let _ = self.spi.send(0x05); // 读状态寄存器
                let status = self.spi_transfer(0x00);
                self.cs.set_high();
                
                if (status & 0x01) == 0 {
                    break;
                }
                delay_ms(1);
            }
        }
        
        fn spi_transfer(&mut self, data: u8) -> u8 {
            nb::block!(self.spi.send(data)).unwrap();
            nb::block!(self.spi.read()).unwrap()
        }
    }
}

// 数据记录模块
mod data_logger {
    use super::*;
    use super::sensors::SensorData;
    use heapless::Vec;
    
    pub struct DataLogger {
        buffer: Vec<SensorData, LOG_BUFFER_SIZE>,
        total_samples: u32,
    }
    
    impl DataLogger {
        pub fn new() -> Self {
            Self {
                buffer: Vec::new(),
                total_samples: 0,
            }
        }
        
        pub fn log_data(&mut self, data: SensorData, sample_count: u32) {
            // 如果缓冲区满了，移除最旧的数据
            if self.buffer.is_full() {
                self.buffer.remove(0);
            }
            
            // 添加新数据
            let _ = self.buffer.push(data);
            self.total_samples = sample_count;
        }
        
        pub fn get_buffer(&self) -> &[SensorData] {
            &self.buffer
        }
        
        pub fn clear_buffer(&mut self) {
            self.buffer.clear();
        }
        
        pub fn get_total_samples(&self) -> u32 {
            self.total_samples
        }
    }
}