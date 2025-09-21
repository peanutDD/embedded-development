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

use sha2::{Sha256, Digest};
use hmac::{Hmac, Mac};
use heapless::{Vec, String};
use signature::{Signature, Signer, Verifier};
use rand_core::{RngCore, CryptoRng};

type HmacSha256 = Hmac<Sha256>;

/// RSA数字签名示例系统
/// 
/// 硬件连接:
/// - USART2: PA2(TX), PA3(RX) - 调试输出
/// - LED: PC13 - 状态指示
/// - 按键: PA0 - 触发签名测试
/// 
/// 功能特性:
/// - RSA密钥生成
/// - RSA-PSS数字签名
/// - RSA-PKCS#1 v1.5签名
/// - 签名验证
/// - 证书验证
/// - 性能测试

/// 系统配置常量
const SYSTEM_CLOCK_HZ: u32 = 84_000_000;
const UART_BAUD_RATE: u32 = 115200;

/// RSA配置
const RSA_KEY_SIZE: usize = 2048;  // RSA-2048
const MAX_MESSAGE_SIZE: usize = 256;
const MAX_SIGNATURE_SIZE: usize = 256;

/// 签名算法类型
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SignatureAlgorithm {
    RsaPss,
    RsaPkcs1v15,
}

/// RSA密钥对
#[derive(Debug, Clone)]
pub struct RsaKeyPair {
    pub public_key: Vec<u8, 512>,
    pub private_key: Vec<u8, 1024>,
    pub key_size: usize,
}

/// 数字签名结果
#[derive(Debug, Clone)]
pub struct SignatureResult {
    pub signature: Vec<u8, MAX_SIGNATURE_SIZE>,
    pub algorithm: SignatureAlgorithm,
    pub message_hash: [u8; 32],
}

/// 简化的RSA实现（仅用于演示）
pub struct SimpleRsa {
    key_pair: Option<RsaKeyPair>,
    algorithm: SignatureAlgorithm,
}

impl SimpleRsa {
    pub fn new(algorithm: SignatureAlgorithm) -> Self {
        Self {
            key_pair: None,
            algorithm,
        }
    }
    
    /// 生成RSA密钥对（简化实现）
    pub fn generate_keypair(&mut self, rng: &mut dyn RngCore) -> Result<(), &'static str> {
        // 注意：这是一个简化的实现，实际应用中应使用完整的RSA密钥生成算法
        let mut public_key = Vec::new();
        let mut private_key = Vec::new();
        
        // 生成模拟的公钥
        for _ in 0..64 {
            let mut bytes = [0u8; 4];
            rng.fill_bytes(&mut bytes);
            for byte in bytes {
                public_key.push(byte).map_err(|_| "Public key too large")?;
            }
        }
        
        // 生成模拟的私钥
        for _ in 0..128 {
            let mut bytes = [0u8; 4];
            rng.fill_bytes(&mut bytes);
            for byte in bytes {
                private_key.push(byte).map_err(|_| "Private key too large")?;
            }
        }
        
        self.key_pair = Some(RsaKeyPair {
            public_key,
            private_key,
            key_size: RSA_KEY_SIZE,
        });
        
        Ok(())
    }
    
    /// 签名消息
    pub fn sign(&self, message: &[u8]) -> Result<SignatureResult, &'static str> {
        let key_pair = self.key_pair.as_ref().ok_or("No key pair available")?;
        
        // 计算消息哈希
        let mut hasher = Sha256::new();
        hasher.update(message);
        let message_hash = hasher.finalize();
        
        let mut hash_array = [0u8; 32];
        hash_array.copy_from_slice(&message_hash);
        
        // 生成签名（简化实现）
        let signature = self.generate_signature(&hash_array, &key_pair.private_key)?;
        
        Ok(SignatureResult {
            signature,
            algorithm: self.algorithm,
            message_hash: hash_array,
        })
    }
    
    /// 验证签名
    pub fn verify(&self, message: &[u8], signature_result: &SignatureResult) -> Result<bool, &'static str> {
        let key_pair = self.key_pair.as_ref().ok_or("No key pair available")?;
        
        // 计算消息哈希
        let mut hasher = Sha256::new();
        hasher.update(message);
        let message_hash = hasher.finalize();
        
        let mut hash_array = [0u8; 32];
        hash_array.copy_from_slice(&message_hash);
        
        // 验证哈希是否匹配
        if hash_array != signature_result.message_hash {
            return Ok(false);
        }
        
        // 验证签名（简化实现）
        self.verify_signature(&hash_array, &signature_result.signature, &key_pair.public_key)
    }
    
    fn generate_signature(&self, hash: &[u8; 32], private_key: &[u8]) -> Result<Vec<u8, MAX_SIGNATURE_SIZE>, &'static str> {
        let mut signature = Vec::new();
        
        match self.algorithm {
            SignatureAlgorithm::RsaPss => {
                // RSA-PSS签名生成（简化实现）
                for i in 0..32 {
                    let sig_byte = hash[i] ^ private_key[i % private_key.len()];
                    signature.push(sig_byte).map_err(|_| "Signature too large")?;
                }
                
                // 添加PSS填充标识
                for i in 0..32 {
                    let padding_byte = 0xBC ^ (i as u8);
                    signature.push(padding_byte).map_err(|_| "Signature too large")?;
                }
            },
            SignatureAlgorithm::RsaPkcs1v15 => {
                // PKCS#1 v1.5签名生成（简化实现）
                // 添加PKCS#1 v1.5填充
                signature.push(0x00).map_err(|_| "Signature too large")?;
                signature.push(0x01).map_err(|_| "Signature too large")?;
                
                // 填充字节
                for _ in 0..10 {
                    signature.push(0xFF).map_err(|_| "Signature too large")?;
                }
                
                signature.push(0x00).map_err(|_| "Signature too large")?;
                
                // DigestInfo for SHA-256
                let digest_info = [
                    0x30, 0x31, 0x30, 0x0d, 0x06, 0x09, 0x60, 0x86,
                    0x48, 0x01, 0x65, 0x03, 0x04, 0x02, 0x01, 0x05,
                    0x00, 0x04, 0x20
                ];
                
                for byte in digest_info {
                    signature.push(byte).map_err(|_| "Signature too large")?;
                }
                
                // 哈希值
                for byte in hash {
                    signature.push(*byte).map_err(|_| "Signature too large")?;
                }
            }
        }
        
        Ok(signature)
    }
    
    fn verify_signature(&self, hash: &[u8; 32], signature: &[u8], public_key: &[u8]) -> Result<bool, &'static str> {
        match self.algorithm {
            SignatureAlgorithm::RsaPss => {
                // RSA-PSS签名验证（简化实现）
                if signature.len() < 64 {
                    return Ok(false);
                }
                
                // 验证哈希部分
                for i in 0..32 {
                    let expected = hash[i] ^ public_key[i % public_key.len()];
                    if signature[i] != expected {
                        return Ok(false);
                    }
                }
                
                // 验证PSS填充
                for i in 0..32 {
                    let expected = 0xBC ^ (i as u8);
                    if signature[32 + i] != expected {
                        return Ok(false);
                    }
                }
                
                Ok(true)
            },
            SignatureAlgorithm::RsaPkcs1v15 => {
                // PKCS#1 v1.5签名验证（简化实现）
                if signature.len() < 51 {
                    return Ok(false);
                }
                
                // 验证填充结构
                if signature[0] != 0x00 || signature[1] != 0x01 {
                    return Ok(false);
                }
                
                // 查找填充结束
                let mut padding_end = 2;
                while padding_end < signature.len() && signature[padding_end] == 0xFF {
                    padding_end += 1;
                }
                
                if padding_end >= signature.len() || signature[padding_end] != 0x00 {
                    return Ok(false);
                }
                
                // 验证DigestInfo
                let digest_info_start = padding_end + 1;
                if digest_info_start + 19 + 32 > signature.len() {
                    return Ok(false);
                }
                
                // 验证哈希值
                let hash_start = digest_info_start + 19;
                for i in 0..32 {
                    if signature[hash_start + i] != hash[i] {
                        return Ok(false);
                    }
                }
                
                Ok(true)
            }
        }
    }
    
    /// 获取公钥
    pub fn get_public_key(&self) -> Option<&Vec<u8, 512>> {
        self.key_pair.as_ref().map(|kp| &kp.public_key)
    }
    
    /// 获取密钥大小
    pub fn get_key_size(&self) -> Option<usize> {
        self.key_pair.as_ref().map(|kp| kp.key_size)
    }
}

/// 证书验证器
pub struct CertificateValidator {
    trusted_keys: Vec<Vec<u8, 512>, 4>,
}

impl CertificateValidator {
    pub fn new() -> Self {
        Self {
            trusted_keys: Vec::new(),
        }
    }
    
    /// 添加受信任的公钥
    pub fn add_trusted_key(&mut self, public_key: Vec<u8, 512>) -> Result<(), &'static str> {
        self.trusted_keys.push(public_key).map_err(|_| "Too many trusted keys")
    }
    
    /// 验证证书链
    pub fn validate_certificate_chain(&self, certificates: &[&[u8]]) -> Result<bool, &'static str> {
        if certificates.is_empty() {
            return Ok(false);
        }
        
        // 简化的证书验证实现
        // 实际应用中需要解析X.509证书格式
        
        // 验证根证书
        let root_cert = certificates.last().unwrap();
        let mut root_key_found = false;
        
        for trusted_key in &self.trusted_keys {
            if self.certificate_matches_key(root_cert, trusted_key.as_slice()) {
                root_key_found = true;
                break;
            }
        }
        
        if !root_key_found {
            return Ok(false);
        }
        
        // 验证证书链
        for i in 0..certificates.len() - 1 {
            if !self.verify_certificate_signature(certificates[i], certificates[i + 1]) {
                return Ok(false);
            }
        }
        
        Ok(true)
    }
    
    fn certificate_matches_key(&self, certificate: &[u8], public_key: &[u8]) -> bool {
        // 简化的证书公钥匹配
        if certificate.len() < public_key.len() {
            return false;
        }
        
        // 查找公钥在证书中的位置（简化实现）
        for i in 0..=certificate.len() - public_key.len() {
            if &certificate[i..i + public_key.len()] == public_key {
                return true;
            }
        }
        
        false
    }
    
    fn verify_certificate_signature(&self, cert: &[u8], issuer_cert: &[u8]) -> bool {
        // 简化的证书签名验证
        // 实际应用中需要解析证书结构并验证数字签名
        
        if cert.len() < 64 || issuer_cert.len() < 64 {
            return false;
        }
        
        // 简单的哈希比较（仅用于演示）
        let mut hasher = Sha256::new();
        hasher.update(&cert[..cert.len() - 32]); // 假设最后32字节是签名
        let cert_hash = hasher.finalize();
        
        let mut issuer_hasher = Sha256::new();
        issuer_hasher.update(&issuer_cert[..32]); // 假设前32字节是公钥的一部分
        issuer_hasher.update(&cert_hash);
        let expected_signature = issuer_hasher.finalize();
        
        // 比较签名
        &cert[cert.len() - 32..] == expected_signature.as_slice()
    }
}

/// 安全随机数生成器包装
pub struct SecureRng {
    rng: Rng,
}

impl SecureRng {
    pub fn new(rng: Rng) -> Self {
        Self { rng }
    }
}

impl RngCore for SecureRng {
    fn next_u32(&mut self) -> u32 {
        self.rng.gen().unwrap_or(0)
    }
    
    fn next_u64(&mut self) -> u64 {
        ((self.next_u32() as u64) << 32) | (self.next_u32() as u64)
    }
    
    fn fill_bytes(&mut self, dest: &mut [u8]) {
        for chunk in dest.chunks_mut(4) {
            let random_u32 = self.next_u32();
            let bytes = random_u32.to_le_bytes();
            
            for (i, &byte) in bytes.iter().enumerate() {
                if i < chunk.len() {
                    chunk[i] = byte;
                }
            }
        }
    }
    
    fn try_fill_bytes(&mut self, dest: &mut [u8]) -> Result<(), rand_core::Error> {
        self.fill_bytes(dest);
        Ok(())
    }
}

impl CryptoRng for SecureRng {}

/// RSA签名测试套件
pub struct RsaSignatureTestSuite {
    rsa_pss: SimpleRsa,
    rsa_pkcs1: SimpleRsa,
    certificate_validator: CertificateValidator,
    rng: SecureRng,
}

impl RsaSignatureTestSuite {
    pub fn new(mut rng: SecureRng) -> Result<Self, &'static str> {
        let mut rsa_pss = SimpleRsa::new(SignatureAlgorithm::RsaPss);
        let mut rsa_pkcs1 = SimpleRsa::new(SignatureAlgorithm::RsaPkcs1v15);
        
        // 生成密钥对
        rsa_pss.generate_keypair(&mut rng)?;
        rsa_pkcs1.generate_keypair(&mut rng)?;
        
        let certificate_validator = CertificateValidator::new();
        
        Ok(Self {
            rsa_pss,
            rsa_pkcs1,
            certificate_validator,
            rng,
        })
    }
    
    /// 测试RSA-PSS签名
    pub fn test_rsa_pss(&self, message: &[u8]) -> Result<bool, &'static str> {
        let signature_result = self.rsa_pss.sign(message)?;
        self.rsa_pss.verify(message, &signature_result)
    }
    
    /// 测试RSA-PKCS#1 v1.5签名
    pub fn test_rsa_pkcs1(&self, message: &[u8]) -> Result<bool, &'static str> {
        let signature_result = self.rsa_pkcs1.sign(message)?;
        self.rsa_pkcs1.verify(message, &signature_result)
    }
    
    /// 测试证书验证
    pub fn test_certificate_validation(&mut self) -> Result<bool, &'static str> {
        // 创建模拟证书
        let mut root_cert = Vec::new();
        let mut intermediate_cert = Vec::new();
        let mut leaf_cert = Vec::new();
        
        // 生成模拟证书数据
        for i in 0..128 {
            root_cert.push(i as u8).map_err(|_| "Certificate too large")?;
            intermediate_cert.push((i + 1) as u8).map_err(|_| "Certificate too large")?;
            leaf_cert.push((i + 2) as u8).map_err(|_| "Certificate too large")?;
        }
        
        // 添加受信任的根证书公钥
        if let Some(public_key) = self.rsa_pss.get_public_key() {
            self.certificate_validator.add_trusted_key(public_key.clone())?;
        }
        
        // 验证证书链
        let certificates = [leaf_cert.as_slice(), intermediate_cert.as_slice(), root_cert.as_slice()];
        self.certificate_validator.validate_certificate_chain(&certificates)
    }
    
    /// 性能测试
    pub fn benchmark_signing(&self, message: &[u8], iterations: u32) -> Result<(u32, u32), &'static str> {
        // RSA-PSS性能测试
        let start_time = cortex_m::peripheral::DWT::cycle_count();
        
        for _ in 0..iterations {
            let _signature = self.rsa_pss.sign(message)?;
        }
        
        let end_time = cortex_m::peripheral::DWT::cycle_count();
        let pss_cycles = end_time.wrapping_sub(start_time) / iterations;
        
        // RSA-PKCS#1 v1.5性能测试
        let start_time = cortex_m::peripheral::DWT::cycle_count();
        
        for _ in 0..iterations {
            let _signature = self.rsa_pkcs1.sign(message)?;
        }
        
        let end_time = cortex_m::peripheral::DWT::cycle_count();
        let pkcs1_cycles = end_time.wrapping_sub(start_time) / iterations;
        
        Ok((pss_cycles, pkcs1_cycles))
    }
}

/// RSA签名系统
pub struct RsaSignatureSystem {
    test_suite: RsaSignatureTestSuite,
    led_pin: stm32f4xx_hal::gpio::gpioc::PC13<stm32f4xx_hal::gpio::Output<stm32f4xx_hal::gpio::PushPull>>,
    button_pin: stm32f4xx_hal::gpio::gpioa::PA0<stm32f4xx_hal::gpio::Input<stm32f4xx_hal::gpio::PullUp>>,
    serial: Serial<stm32::USART2>,
}

impl RsaSignatureSystem {
    pub fn new(
        test_suite: RsaSignatureTestSuite,
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
        self.print_message("RSA Signature System Initialized\r\n");
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
                
                self.run_signature_tests();
                
                self.led_pin.set_high();
            } else if self.button_pin.is_high() {
                button_pressed = false;
            }
            
            // 短暂延时
            cortex_m::asm::delay(SYSTEM_CLOCK_HZ / 1000); // 1ms
        }
    }
    
    fn run_signature_tests(&mut self) {
        self.print_message("Starting RSA Signature Tests...\r\n");
        
        // 测试消息
        let test_message = b"Hello, RSA Digital Signature! This is a test message for signing.";
        
        // RSA-PSS测试
        self.print_message("Testing RSA-PSS Signature...\r\n");
        match self.test_suite.test_rsa_pss(test_message) {
            Ok(true) => self.print_message("RSA-PSS Test: PASSED\r\n"),
            Ok(false) => self.print_message("RSA-PSS Test: FAILED\r\n"),
            Err(e) => {
                self.print_message("RSA-PSS Test Error: ");
                self.print_message(e);
                self.print_message("\r\n");
            }
        }
        
        // RSA-PKCS#1 v1.5测试
        self.print_message("Testing RSA-PKCS#1 v1.5 Signature...\r\n");
        match self.test_suite.test_rsa_pkcs1(test_message) {
            Ok(true) => self.print_message("RSA-PKCS#1 Test: PASSED\r\n"),
            Ok(false) => self.print_message("RSA-PKCS#1 Test: FAILED\r\n"),
            Err(e) => {
                self.print_message("RSA-PKCS#1 Test Error: ");
                self.print_message(e);
                self.print_message("\r\n");
            }
        }
        
        // 证书验证测试
        self.print_message("Testing Certificate Validation...\r\n");
        match self.test_suite.test_certificate_validation() {
            Ok(true) => self.print_message("Certificate Validation Test: PASSED\r\n"),
            Ok(false) => self.print_message("Certificate Validation Test: FAILED\r\n"),
            Err(e) => {
                self.print_message("Certificate Validation Test Error: ");
                self.print_message(e);
                self.print_message("\r\n");
            }
        }
        
        // 性能测试
        self.print_message("Running Performance Test...\r\n");
        match self.test_suite.benchmark_signing(b"Performance test", 5) {
            Ok((pss_cycles, pkcs1_cycles)) => {
                self.print_message("RSA-PSS cycles per signature: ");
                self.print_number(pss_cycles);
                self.print_message("\r\n");
                
                self.print_message("RSA-PKCS#1 cycles per signature: ");
                self.print_number(pkcs1_cycles);
                self.print_message("\r\n");
            },
            Err(e) => {
                self.print_message("Performance Test Error: ");
                self.print_message(e);
                self.print_message("\r\n");
            }
        }
        
        self.print_message("All signature tests completed!\r\n\r\n");
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
    let test_suite = RsaSignatureTestSuite::new(secure_rng).unwrap();
    
    // 创建系统
    let mut system = RsaSignatureSystem::new(
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