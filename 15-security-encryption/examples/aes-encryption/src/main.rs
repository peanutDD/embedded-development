#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m_rt::entry;
use stm32f4xx_hal::{
    prelude::*,
    stm32,
    serial::{config::Config, Serial},
    rng::Rng,
};

use aes::Aes128;
use aes::cipher::{BlockEncrypt, BlockDecrypt, KeyInit};
use sha2::{Sha256, Digest};
use hmac::{Hmac, Mac};
use heapless::{Vec, String};

type HmacSha256 = Hmac<Sha256>;

/// AES加密示例系统
/// 
/// 硬件连接:
/// - USART2: PA2(TX), PA3(RX) - 调试输出
/// - LED: PC13 - 状态指示
/// - 按键: PA0 - 触发加密测试
/// 
/// 功能特性:
/// - AES-128 ECB/CBC模式加密
/// - HMAC-SHA256消息认证
/// - 安全随机数生成
/// - 密钥派生功能
/// - 加密性能测试

/// 系统配置常量
const SYSTEM_CLOCK_HZ: u32 = 84_000_000;
const UART_BAUD_RATE: u32 = 115200;

/// 加密配置
const AES_KEY_SIZE: usize = 16;  // AES-128
const AES_BLOCK_SIZE: usize = 16;
const HMAC_KEY_SIZE: usize = 32;
const MAX_DATA_SIZE: usize = 256;

/// 加密模式
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum EncryptionMode {
    ECB,
    CBC,
}

/// 加密结果
#[derive(Debug, Clone)]
pub struct EncryptionResult {
    pub ciphertext: Vec<u8, MAX_DATA_SIZE>,
    pub iv: Option<[u8; AES_BLOCK_SIZE]>,
    pub hmac: [u8; 32],
}

/// AES加密器
pub struct AesEncryptor {
    key: [u8; AES_KEY_SIZE],
    hmac_key: [u8; HMAC_KEY_SIZE],
    mode: EncryptionMode,
}

impl AesEncryptor {
    pub fn new(key: [u8; AES_KEY_SIZE], hmac_key: [u8; HMAC_KEY_SIZE], mode: EncryptionMode) -> Self {
        Self {
            key,
            hmac_key,
            mode,
        }
    }
    
    /// 加密数据
    pub fn encrypt(&self, plaintext: &[u8], iv: Option<[u8; AES_BLOCK_SIZE]>) -> Result<EncryptionResult, &'static str> {
        match self.mode {
            EncryptionMode::ECB => self.encrypt_ecb(plaintext),
            EncryptionMode::CBC => {
                let iv = iv.ok_or("CBC mode requires IV")?;
                self.encrypt_cbc(plaintext, iv)
            }
        }
    }
    
    /// 解密数据
    pub fn decrypt(&self, result: &EncryptionResult) -> Result<Vec<u8, MAX_DATA_SIZE>, &'static str> {
        // 验证HMAC
        if !self.verify_hmac(&result.ciphertext, &result.hmac) {
            return Err("HMAC verification failed");
        }
        
        match self.mode {
            EncryptionMode::ECB => self.decrypt_ecb(&result.ciphertext),
            EncryptionMode::CBC => {
                let iv = result.iv.ok_or("CBC mode requires IV")?;
                self.decrypt_cbc(&result.ciphertext, iv)
            }
        }
    }
    
    fn encrypt_ecb(&self, plaintext: &[u8]) -> Result<EncryptionResult, &'static str> {
        let cipher = Aes128::new_from_slice(&self.key).map_err(|_| "Invalid key")?;
        
        // 添加PKCS#7填充
        let padded_data = self.add_pkcs7_padding(plaintext)?;
        
        let mut ciphertext = Vec::new();
        
        // ECB模式加密
        for chunk in padded_data.chunks(AES_BLOCK_SIZE) {
            let mut block = [0u8; AES_BLOCK_SIZE];
            block.copy_from_slice(chunk);
            
            let mut block_array = block.into();
            cipher.encrypt_block(&mut block_array);
            let encrypted_block: [u8; AES_BLOCK_SIZE] = block_array.into();
            
            ciphertext.extend_from_slice(&encrypted_block).map_err(|_| "Ciphertext too large")?;
        }
        
        // 计算HMAC
        let hmac = self.calculate_hmac(&ciphertext);
        
        Ok(EncryptionResult {
            ciphertext,
            iv: None,
            hmac,
        })
    }
    
    fn encrypt_cbc(&self, plaintext: &[u8], iv: [u8; AES_BLOCK_SIZE]) -> Result<EncryptionResult, &'static str> {
        let cipher = Aes128::new_from_slice(&self.key).map_err(|_| "Invalid key")?;
        
        // 添加PKCS#7填充
        let padded_data = self.add_pkcs7_padding(plaintext)?;
        
        let mut ciphertext = Vec::new();
        let mut prev_block = iv;
        
        // CBC模式加密
        for chunk in padded_data.chunks(AES_BLOCK_SIZE) {
            let mut block = [0u8; AES_BLOCK_SIZE];
            block.copy_from_slice(chunk);
            
            // XOR with previous block
            for i in 0..AES_BLOCK_SIZE {
                block[i] ^= prev_block[i];
            }
            
            let mut block_array = block.into();
            cipher.encrypt_block(&mut block_array);
            let encrypted_block: [u8; AES_BLOCK_SIZE] = block_array.into();
            
            ciphertext.extend_from_slice(&encrypted_block).map_err(|_| "Ciphertext too large")?;
            prev_block = encrypted_block;
        }
        
        // 计算HMAC
        let hmac = self.calculate_hmac(&ciphertext);
        
        Ok(EncryptionResult {
            ciphertext,
            iv: Some(iv),
            hmac,
        })
    }
    
    fn decrypt_ecb(&self, ciphertext: &[u8]) -> Result<Vec<u8, MAX_DATA_SIZE>, &'static str> {
        if ciphertext.len() % AES_BLOCK_SIZE != 0 {
            return Err("Invalid ciphertext length");
        }
        
        let cipher = Aes128::new_from_slice(&self.key).map_err(|_| "Invalid key")?;
        
        let mut plaintext = Vec::new();
        
        // ECB模式解密
        for chunk in ciphertext.chunks(AES_BLOCK_SIZE) {
            let mut block = [0u8; AES_BLOCK_SIZE];
            block.copy_from_slice(chunk);
            
            let mut block_array = block.into();
            cipher.decrypt_block(&mut block_array);
            let decrypted_block: [u8; AES_BLOCK_SIZE] = block_array.into();
            
            plaintext.extend_from_slice(&decrypted_block).map_err(|_| "Plaintext too large")?;
        }
        
        // 移除PKCS#7填充
        self.remove_pkcs7_padding(&mut plaintext)?;
        
        Ok(plaintext)
    }
    
    fn decrypt_cbc(&self, ciphertext: &[u8], iv: [u8; AES_BLOCK_SIZE]) -> Result<Vec<u8, MAX_DATA_SIZE>, &'static str> {
        if ciphertext.len() % AES_BLOCK_SIZE != 0 {
            return Err("Invalid ciphertext length");
        }
        
        let cipher = Aes128::new_from_slice(&self.key).map_err(|_| "Invalid key")?;
        
        let mut plaintext = Vec::new();
        let mut prev_block = iv;
        
        // CBC模式解密
        for chunk in ciphertext.chunks(AES_BLOCK_SIZE) {
            let mut block = [0u8; AES_BLOCK_SIZE];
            block.copy_from_slice(chunk);
            
            let current_cipher = block;
            
            let mut block_array = block.into();
            cipher.decrypt_block(&mut block_array);
            let mut decrypted_block: [u8; AES_BLOCK_SIZE] = block_array.into();
            
            // XOR with previous block
            for i in 0..AES_BLOCK_SIZE {
                decrypted_block[i] ^= prev_block[i];
            }
            
            plaintext.extend_from_slice(&decrypted_block).map_err(|_| "Plaintext too large")?;
            prev_block = current_cipher;
        }
        
        // 移除PKCS#7填充
        self.remove_pkcs7_padding(&mut plaintext)?;
        
        Ok(plaintext)
    }
    
    fn add_pkcs7_padding(&self, data: &[u8]) -> Result<Vec<u8, MAX_DATA_SIZE>, &'static str> {
        let mut padded = Vec::new();
        padded.extend_from_slice(data).map_err(|_| "Data too large")?;
        
        let padding_len = AES_BLOCK_SIZE - (data.len() % AES_BLOCK_SIZE);
        
        for _ in 0..padding_len {
            padded.push(padding_len as u8).map_err(|_| "Padding failed")?;
        }
        
        Ok(padded)
    }
    
    fn remove_pkcs7_padding(&self, data: &mut Vec<u8, MAX_DATA_SIZE>) -> Result<(), &'static str> {
        if data.is_empty() {
            return Err("Empty data");
        }
        
        let padding_len = *data.last().unwrap() as usize;
        
        if padding_len == 0 || padding_len > AES_BLOCK_SIZE {
            return Err("Invalid padding");
        }
        
        if data.len() < padding_len {
            return Err("Invalid padding length");
        }
        
        // 验证填充
        let start_pos = data.len() - padding_len;
        for i in start_pos..data.len() {
            if data[i] != padding_len as u8 {
                return Err("Invalid padding bytes");
            }
        }
        
        data.truncate(start_pos);
        
        Ok(())
    }
    
    fn calculate_hmac(&self, data: &[u8]) -> [u8; 32] {
        let mut mac = HmacSha256::new_from_slice(&self.hmac_key).expect("Valid key length");
        mac.update(data);
        let result = mac.finalize().into_bytes();
        
        let mut hmac = [0u8; 32];
        hmac.copy_from_slice(&result);
        hmac
    }
    
    fn verify_hmac(&self, data: &[u8], expected_hmac: &[u8; 32]) -> bool {
        let calculated_hmac = self.calculate_hmac(data);
        calculated_hmac == *expected_hmac
    }
}

/// 密钥派生函数
pub struct KeyDerivation;

impl KeyDerivation {
    /// PBKDF2密钥派生
    pub fn pbkdf2(password: &[u8], salt: &[u8], iterations: u32, key_len: usize) -> Vec<u8, 64> {
        // 简化的PBKDF2实现
        let mut derived_key = Vec::new();
        
        let mut hasher = Sha256::new();
        hasher.update(password);
        hasher.update(salt);
        hasher.update(&iterations.to_be_bytes());
        
        let hash = hasher.finalize();
        
        for i in 0..key_len.min(32).min(64) {
            derived_key.push(hash[i]).ok();
        }
        
        derived_key
    }
    
    /// HKDF密钥派生
    pub fn hkdf(ikm: &[u8], salt: &[u8], info: &[u8], length: usize) -> Vec<u8, 64> {
        // 简化的HKDF实现
        let mut derived_key = Vec::new();
        
        // Extract phase
        let mut mac = HmacSha256::new_from_slice(salt).expect("Valid salt");
        mac.update(ikm);
        let prk = mac.finalize().into_bytes();
        
        // Expand phase
        let mut hasher = Sha256::new();
        hasher.update(&prk);
        hasher.update(info);
        hasher.update(&[0x01]); // Counter
        
        let hash = hasher.finalize();
        
        for i in 0..length.min(32).min(64) {
            derived_key.push(hash[i]).ok();
        }
        
        derived_key
    }
}

/// 安全随机数生成器
pub struct SecureRng {
    rng: Rng,
}

impl SecureRng {
    pub fn new(rng: Rng) -> Self {
        Self { rng }
    }
    
    /// 生成随机字节
    pub fn generate_bytes(&mut self, buffer: &mut [u8]) -> Result<(), &'static str> {
        for byte in buffer.iter_mut() {
            *byte = self.rng.gen().map_err(|_| "RNG error")? as u8;
        }
        Ok(())
    }
    
    /// 生成AES密钥
    pub fn generate_aes_key(&mut self) -> Result<[u8; AES_KEY_SIZE], &'static str> {
        let mut key = [0u8; AES_KEY_SIZE];
        self.generate_bytes(&mut key)?;
        Ok(key)
    }
    
    /// 生成HMAC密钥
    pub fn generate_hmac_key(&mut self) -> Result<[u8; HMAC_KEY_SIZE], &'static str> {
        let mut key = [0u8; HMAC_KEY_SIZE];
        self.generate_bytes(&mut key)?;
        Ok(key)
    }
    
    /// 生成初始化向量
    pub fn generate_iv(&mut self) -> Result<[u8; AES_BLOCK_SIZE], &'static str> {
        let mut iv = [0u8; AES_BLOCK_SIZE];
        self.generate_bytes(&mut iv)?;
        Ok(iv)
    }
}

/// 加密测试套件
pub struct EncryptionTestSuite {
    encryptor_ecb: AesEncryptor,
    encryptor_cbc: AesEncryptor,
    rng: SecureRng,
}

impl EncryptionTestSuite {
    pub fn new(mut rng: SecureRng) -> Result<Self, &'static str> {
        let aes_key = rng.generate_aes_key()?;
        let hmac_key = rng.generate_hmac_key()?;
        
        let encryptor_ecb = AesEncryptor::new(aes_key, hmac_key, EncryptionMode::ECB);
        let encryptor_cbc = AesEncryptor::new(aes_key, hmac_key, EncryptionMode::CBC);
        
        Ok(Self {
            encryptor_ecb,
            encryptor_cbc,
            rng,
        })
    }
    
    /// 运行ECB模式测试
    pub fn test_ecb_mode(&self, data: &[u8]) -> Result<bool, &'static str> {
        // 加密
        let encrypted = self.encryptor_ecb.encrypt(data, None)?;
        
        // 解密
        let decrypted = self.encryptor_ecb.decrypt(&encrypted)?;
        
        // 验证
        Ok(decrypted.as_slice() == data)
    }
    
    /// 运行CBC模式测试
    pub fn test_cbc_mode(&mut self, data: &[u8]) -> Result<bool, &'static str> {
        let iv = self.rng.generate_iv()?;
        
        // 加密
        let encrypted = self.encryptor_cbc.encrypt(data, Some(iv))?;
        
        // 解密
        let decrypted = self.encryptor_cbc.decrypt(&encrypted)?;
        
        // 验证
        Ok(decrypted.as_slice() == data)
    }
    
    /// 运行密钥派生测试
    pub fn test_key_derivation(&self) -> Result<bool, &'static str> {
        let password = b"test_password";
        let salt = b"test_salt_123456";
        
        // PBKDF2测试
        let pbkdf2_key = KeyDerivation::pbkdf2(password, salt, 1000, 32);
        
        // HKDF测试
        let hkdf_key = KeyDerivation::hkdf(password, salt, b"test_info", 32);
        
        // 验证密钥不同
        Ok(pbkdf2_key.as_slice() != hkdf_key.as_slice())
    }
    
    /// 性能测试
    pub fn benchmark_encryption(&mut self, data: &[u8], iterations: u32) -> Result<u32, &'static str> {
        let start_time = cortex_m::peripheral::DWT::cycle_count();
        
        for _ in 0..iterations {
            let iv = self.rng.generate_iv()?;
            let encrypted = self.encryptor_cbc.encrypt(data, Some(iv))?;
            let _decrypted = self.encryptor_cbc.decrypt(&encrypted)?;
        }
        
        let end_time = cortex_m::peripheral::DWT::cycle_count();
        let cycles = end_time.wrapping_sub(start_time);
        
        Ok(cycles / iterations)
    }
}

/// AES加密系统
pub struct AesEncryptionSystem {
    test_suite: EncryptionTestSuite,
    led_pin: stm32f4xx_hal::gpio::gpioc::PC13<stm32f4xx_hal::gpio::Output<stm32f4xx_hal::gpio::PushPull>>,
    button_pin: stm32f4xx_hal::gpio::gpioa::PA0<stm32f4xx_hal::gpio::Input<stm32f4xx_hal::gpio::PullUp>>,
    serial: Serial<stm32::USART2>,
}

impl AesEncryptionSystem {
    pub fn new(
        test_suite: EncryptionTestSuite,
        led_pin: stm32f4xx_hal::gpio::gpioc::PC13<stm32f4xx_hal::gpio::Output<stm32f4xx_hal::gpio::PushPull>>,
        button_pin: stm32f4xx_hal::gpio::gpioa::PA0<stm32f4xx_hal::gpio::Input<stm32f4xx_hal::gpio::PullUp>>,
        serial: Serial<stm32::USART2>,
    ) -> Self {
        Self {
            test_suite,
            led_pin,
            button_pin,
            serial,
        }
    }
    
    /// 系统初始化
    pub fn init(&mut self) -> Result<(), &'static str> {
        self.led_pin.set_high();
        self.print_message("AES Encryption System Initialized\r\n");
        Ok(())
    }
    
    /// 主循环
    pub fn run(&mut self) -> ! {
        let mut button_pressed = false;
        
        loop {
            // 检查按键
            if self.button_pin.is_low() && !button_pressed {
                button_pressed = true;
                self.led_pin.set_low();
                
                self.run_encryption_tests();
                
                self.led_pin.set_high();
            } else if self.button_pin.is_high() {
                button_pressed = false;
            }
            
            // 短暂延时
            cortex_m::asm::delay(SYSTEM_CLOCK_HZ / 1000); // 1ms
        }
    }
    
    fn run_encryption_tests(&mut self) {
        self.print_message("Starting Encryption Tests...\r\n");
        
        // 测试数据
        let test_data = b"Hello, AES Encryption World! This is a test message for encryption.";
        
        // ECB模式测试
        self.print_message("Testing ECB Mode...\r\n");
        match self.test_suite.test_ecb_mode(test_data) {
            Ok(true) => self.print_message("ECB Test: PASSED\r\n"),
            Ok(false) => self.print_message("ECB Test: FAILED\r\n"),
            Err(e) => {
                self.print_message("ECB Test Error: ");
                self.print_message(e);
                self.print_message("\r\n");
            }
        }
        
        // CBC模式测试
        self.print_message("Testing CBC Mode...\r\n");
        match self.test_suite.test_cbc_mode(test_data) {
            Ok(true) => self.print_message("CBC Test: PASSED\r\n"),
            Ok(false) => self.print_message("CBC Test: FAILED\r\n"),
            Err(e) => {
                self.print_message("CBC Test Error: ");
                self.print_message(e);
                self.print_message("\r\n");
            }
        }
        
        // 密钥派生测试
        self.print_message("Testing Key Derivation...\r\n");
        match self.test_suite.test_key_derivation() {
            Ok(true) => self.print_message("Key Derivation Test: PASSED\r\n"),
            Ok(false) => self.print_message("Key Derivation Test: FAILED\r\n"),
            Err(e) => {
                self.print_message("Key Derivation Test Error: ");
                self.print_message(e);
                self.print_message("\r\n");
            }
        }
        
        // 性能测试
        self.print_message("Running Performance Test...\r\n");
        match self.test_suite.benchmark_encryption(b"Performance test data", 10) {
            Ok(cycles) => {
                self.print_message("Average cycles per encryption: ");
                self.print_number(cycles);
                self.print_message("\r\n");
            },
            Err(e) => {
                self.print_message("Performance Test Error: ");
                self.print_message(e);
                self.print_message("\r\n");
            }
        }
        
        self.print_message("All tests completed!\r\n\r\n");
    }
    
    fn print_message(&mut self, msg: &str) {
        for byte in msg.bytes() {
            nb::block!(self.serial.write(byte)).ok();
        }
    }
    
    fn print_number(&mut self, num: u32) {
        let mut buffer = [0u8; 10];
        let mut temp = num;
        let mut pos = 0;
        
        if temp == 0 {
            buffer[0] = b'0';
            pos = 1;
        } else {
            while temp > 0 {
                buffer[pos] = (temp % 10) as u8 + b'0';
                temp /= 10;
                pos += 1;
            }
        }
        
        // 反转数字
        for i in 0..pos/2 {
            buffer.swap(i, pos - 1 - i);
        }
        
        for i in 0..pos {
            nb::block!(self.serial.write(buffer[i])).ok();
        }
    }
}

#[entry]
fn main() -> ! {
    // 获取设备外设
    let dp = stm32::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();
    
    // 配置时钟
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr
        .sysclk(SYSTEM_CLOCK_HZ.Hz())
        .freeze();
    
    // 启用DWT循环计数器
    let mut dwt = cp.DWT;
    dwt.enable_cycle_counter();
    
    // 配置GPIO
    let gpioa = dp.GPIOA.split();
    let gpioc = dp.GPIOC.split();
    
    // LED引脚 (PC13)
    let led_pin = gpioc.pc13.into_push_pull_output();
    
    // 按键引脚 (PA0)
    let button_pin = gpioa.pa0.into_pull_up_input();
    
    // 配置UART
    let tx_pin = gpioa.pa2.into_alternate();
    let rx_pin = gpioa.pa3.into_alternate();
    
    let serial = Serial::new(
        dp.USART2,
        (tx_pin, rx_pin),
        Config::default().baudrate(UART_BAUD_RATE.bps()),
        &clocks,
    ).unwrap();
    
    // 初始化RNG
    let rng = Rng::new(dp.RNG, &clocks);
    let secure_rng = SecureRng::new(rng);
    
    // 创建测试套件
    let test_suite = EncryptionTestSuite::new(secure_rng).unwrap();
    
    // 创建系统
    let mut system = AesEncryptionSystem::new(
        test_suite,
        led_pin,
        button_pin,
        serial,
    );
    
    // 初始化系统
    system.init().unwrap();
    
    // 运行主循环
    system.run()
}