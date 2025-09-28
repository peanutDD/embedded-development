#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m_rt::entry;
use stm32f4xx_hal::{
  flash::{Flash, FlashExt},
  prelude::*,
  rng::Rng,
  serial::{config::Config, Serial},
  stm32,
};

use aes::cipher::{BlockDecrypt, KeyInit};
use aes::Aes256;
use heapless::{String, Vec};
use hmac::{Hmac, Mac};
use sha2::{Digest, Sha256};
use signature::{Signature, Verifier};

type HmacSha256 = Hmac<Sha256>;

/// 安全引导加载器系统
///
/// 硬件连接:
/// - USART2: PA2(TX), PA3(RX) - 调试输出
/// - LED: PC13 - 状态指示
/// - 按键: PA0 - 强制进入更新模式
/// - Flash: 内部Flash存储
///
/// 功能特性:
/// - 安全启动验证
/// - 固件签名验证
/// - 加密固件解密
/// - 回滚保护
/// - 安全固件更新
/// - 密钥管理

/// 系统配置常量
const SYSTEM_CLOCK_HZ: u32 = 84_000_000;
const UART_BAUD_RATE: u32 = 115200;

/// 内存布局配置
const BOOTLOADER_START: u32 = 0x08000000;
const BOOTLOADER_SIZE: u32 = 0x10000; // 64KB
const APPLICATION_START: u32 = 0x08010000;
const APPLICATION_SIZE: u32 = 0xF0000; // 960KB
const BACKUP_START: u32 = 0x08100000; // 备份区域（如果有外部Flash）

/// 安全配置
const SIGNATURE_SIZE: usize = 64; // Ed25519签名大小
const AES_KEY_SIZE: usize = 32; // AES-256密钥大小
const AES_BLOCK_SIZE: usize = 16;
const HASH_SIZE: usize = 32; // SHA-256哈希大小
const MAX_FIRMWARE_SIZE: usize = 0xF0000; // 最大固件大小

/// 固件头部结构
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct FirmwareHeader {
  pub magic: u32,                      // 魔数标识
  pub version: u32,                    // 固件版本
  pub size: u32,                       // 固件大小
  pub crc32: u32,                      // CRC32校验
  pub signature: [u8; SIGNATURE_SIZE], // 数字签名
  pub encrypted: u32,                  // 是否加密
  pub timestamp: u64,                  // 时间戳
  pub hash: [u8; HASH_SIZE],           // SHA-256哈希
}

impl FirmwareHeader {
  const MAGIC: u32 = 0xDEADBEEF;

  pub fn new() -> Self {
    Self {
      magic: Self::MAGIC,
      version: 0,
      size: 0,
      crc32: 0,
      signature: [0; SIGNATURE_SIZE],
      encrypted: 0,
      timestamp: 0,
      hash: [0; HASH_SIZE],
    }
  }

  pub fn is_valid(&self) -> bool {
    self.magic == Self::MAGIC && self.size > 0 && self.size <= MAX_FIRMWARE_SIZE as u32
  }
}

/// 安全配置结构
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct SecurityConfig {
  pub public_key: [u8; 32],        // Ed25519公钥
  pub aes_key: [u8; AES_KEY_SIZE], // AES解密密钥
  pub rollback_version: u32,       // 最小允许版本
  pub secure_boot_enabled: u32,    // 安全启动使能
  pub encryption_enabled: u32,     // 加密使能
  pub debug_disabled: u32,         // 调试禁用
}

impl SecurityConfig {
  pub fn new() -> Self {
    Self {
      public_key: [0; 32],
      aes_key: [0; AES_KEY_SIZE],
      rollback_version: 0,
      secure_boot_enabled: 1,
      encryption_enabled: 0,
      debug_disabled: 0,
    }
  }
}

/// 启动状态
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BootStatus {
  Success,
  InvalidHeader,
  SignatureVerificationFailed,
  DecryptionFailed,
  RollbackProtectionFailed,
  CorruptedFirmware,
  UpdateRequired,
}

/// 签名验证器（简化的Ed25519实现）
pub struct SignatureVerifier {
  public_key: [u8; 32],
}

impl SignatureVerifier {
  pub fn new(public_key: [u8; 32]) -> Self {
    Self { public_key }
  }

  /// 验证Ed25519签名（简化实现）
  pub fn verify(&self, message: &[u8], signature: &[u8; SIGNATURE_SIZE]) -> bool {
    // 注意：这是一个简化的实现，实际应用中应使用完整的Ed25519验证算法

    // 计算消息哈希
    let mut hasher = Sha256::new();
    hasher.update(message);
    hasher.update(&self.public_key);
    let hash = hasher.finalize();

    // 简化的签名验证
    let mut expected_signature = [0u8; SIGNATURE_SIZE];
    for i in 0..32 {
      expected_signature[i] = hash[i];
      expected_signature[i + 32] = hash[i] ^ self.public_key[i];
    }

    // 常数时间比较
    let mut result = 0u8;
    for i in 0..SIGNATURE_SIZE {
      result |= signature[i] ^ expected_signature[i];
    }

    result == 0
  }
}

/// 固件解密器
pub struct FirmwareDecryptor {
  key: [u8; AES_KEY_SIZE],
}

impl FirmwareDecryptor {
  pub fn new(key: [u8; AES_KEY_SIZE]) -> Self {
    Self { key }
  }

  /// 解密固件数据
  pub fn decrypt(
    &self,
    encrypted_data: &[u8],
    iv: &[u8; AES_BLOCK_SIZE],
  ) -> Result<Vec<u8, MAX_FIRMWARE_SIZE>, &'static str> {
    if encrypted_data.len() % AES_BLOCK_SIZE != 0 {
      return Err("Invalid encrypted data length");
    }

    let cipher = Aes256::new_from_slice(&self.key).map_err(|_| "Invalid key")?;

    let mut decrypted = Vec::new();
    let mut prev_block = *iv;

    // CBC模式解密
    for chunk in encrypted_data.chunks(AES_BLOCK_SIZE) {
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

      decrypted
        .extend_from_slice(&decrypted_block)
        .map_err(|_| "Decrypted data too large")?;
      prev_block = current_cipher;
    }

    Ok(decrypted)
  }
}

/// Flash管理器
pub struct FlashManager {
  flash: Flash,
}

impl FlashManager {
  pub fn new(flash: Flash) -> Self {
    Self { flash }
  }

  /// 读取固件头部
  pub fn read_firmware_header(&self, address: u32) -> Result<FirmwareHeader, &'static str> {
    // 简化的Flash读取实现
    // 实际应用中需要使用HAL的Flash读取功能

    let mut header = FirmwareHeader::new();

    // 模拟从Flash读取头部数据
    unsafe {
      let flash_ptr = address as *const u8;
      let header_ptr = &mut header as *mut FirmwareHeader as *mut u8;

      for i in 0..core::mem::size_of::<FirmwareHeader>() {
        *header_ptr.add(i) = *flash_ptr.add(i);
      }
    }

    if header.is_valid() {
      Ok(header)
    } else {
      Err("Invalid firmware header")
    }
  }

  /// 读取固件数据
  pub fn read_firmware_data(
    &self,
    address: u32,
    size: u32,
  ) -> Result<Vec<u8, MAX_FIRMWARE_SIZE>, &'static str> {
    if size > MAX_FIRMWARE_SIZE as u32 {
      return Err("Firmware too large");
    }

    let mut data = Vec::new();

    // 模拟从Flash读取数据
    unsafe {
      let flash_ptr = address as *const u8;

      for i in 0..size as usize {
        data.push(*flash_ptr.add(i)).map_err(|_| "Data too large")?;
      }
    }

    Ok(data)
  }

  /// 写入固件数据
  pub fn write_firmware(&mut self, address: u32, data: &[u8]) -> Result<(), &'static str> {
    // 简化的Flash写入实现
    // 实际应用中需要使用HAL的Flash写入功能

    if data.len() > MAX_FIRMWARE_SIZE {
      return Err("Firmware too large");
    }

    // 擦除Flash扇区
    // self.flash.erase_sector(...)?;

    // 写入数据
    // self.flash.write(address, data)?;

    Ok(())
  }

  /// 验证固件完整性
  pub fn verify_firmware_integrity(&self, header: &FirmwareHeader, data: &[u8]) -> bool {
    // 验证大小
    if data.len() != header.size as usize {
      return false;
    }

    // 验证CRC32
    let calculated_crc = self.calculate_crc32(data);
    if calculated_crc != header.crc32 {
      return false;
    }

    // 验证SHA-256哈希
    let mut hasher = Sha256::new();
    hasher.update(data);
    let calculated_hash = hasher.finalize();

    if calculated_hash.as_slice() != &header.hash {
      return false;
    }

    true
  }

  fn calculate_crc32(&self, data: &[u8]) -> u32 {
    // 简化的CRC32计算
    let mut crc = 0xFFFFFFFFu32;

    for &byte in data {
      crc ^= byte as u32;
      for _ in 0..8 {
        if crc & 1 != 0 {
          crc = (crc >> 1) ^ 0xEDB88320;
        } else {
          crc >>= 1;
        }
      }
    }

    !crc
  }
}

/// 安全引导加载器
pub struct SecureBootloader {
  flash_manager: FlashManager,
  signature_verifier: SignatureVerifier,
  firmware_decryptor: FirmwareDecryptor,
  security_config: SecurityConfig,
  serial: Serial<stm32::USART2>,
}

impl SecureBootloader {
  pub fn new(
    flash_manager: FlashManager,
    signature_verifier: SignatureVerifier,
    firmware_decryptor: FirmwareDecryptor,
    security_config: SecurityConfig,
    serial: Serial<stm32::USART2>,
  ) -> Self {
    Self {
      flash_manager,
      signature_verifier,
      firmware_decryptor,
      security_config,
      serial,
    }
  }

  /// 安全启动流程
  pub fn secure_boot(&mut self) -> BootStatus {
    self.print_message("Starting Secure Boot Process...\r\n");

    // 1. 读取固件头部
    let header = match self.flash_manager.read_firmware_header(APPLICATION_START) {
      Ok(header) => header,
      Err(_) => {
        self.print_message("Error: Invalid firmware header\r\n");
        return BootStatus::InvalidHeader;
      }
    };

    self.print_message("Firmware header loaded\r\n");

    // 2. 回滚保护检查
    if self.security_config.secure_boot_enabled != 0 {
      if header.version < self.security_config.rollback_version {
        self.print_message("Error: Rollback protection failed\r\n");
        return BootStatus::RollbackProtectionFailed;
      }
    }

    // 3. 读取固件数据
    let firmware_data = match self.flash_manager.read_firmware_data(
      APPLICATION_START + core::mem::size_of::<FirmwareHeader>() as u32,
      header.size,
    ) {
      Ok(data) => data,
      Err(_) => {
        self.print_message("Error: Failed to read firmware data\r\n");
        return BootStatus::CorruptedFirmware;
      }
    };

    // 4. 验证固件完整性
    if !self
      .flash_manager
      .verify_firmware_integrity(&header, &firmware_data)
    {
      self.print_message("Error: Firmware integrity check failed\r\n");
      return BootStatus::CorruptedFirmware;
    }

    self.print_message("Firmware integrity verified\r\n");

    // 5. 验证数字签名
    if self.security_config.secure_boot_enabled != 0 {
      if !self
        .signature_verifier
        .verify(&firmware_data, &header.signature)
      {
        self.print_message("Error: Signature verification failed\r\n");
        return BootStatus::SignatureVerificationFailed;
      }

      self.print_message("Digital signature verified\r\n");
    }

    // 6. 解密固件（如果需要）
    let final_firmware = if header.encrypted != 0 && self.security_config.encryption_enabled != 0 {
      self.print_message("Decrypting firmware...\r\n");

      // 使用时间戳作为IV（简化实现）
      let mut iv = [0u8; AES_BLOCK_SIZE];
      let timestamp_bytes = header.timestamp.to_le_bytes();
      for i in 0..8 {
        iv[i] = timestamp_bytes[i];
        iv[i + 8] = timestamp_bytes[i];
      }

      match self.firmware_decryptor.decrypt(&firmware_data, &iv) {
        Ok(decrypted) => {
          self.print_message("Firmware decrypted successfully\r\n");
          decrypted
        }
        Err(_) => {
          self.print_message("Error: Firmware decryption failed\r\n");
          return BootStatus::DecryptionFailed;
        }
      }
    } else {
      firmware_data
    };

    // 7. 跳转到应用程序
    self.print_message("Secure boot completed successfully\r\n");
    self.print_message("Jumping to application...\r\n");

    // 实际跳转到应用程序
    self.jump_to_application(APPLICATION_START);

    BootStatus::Success
  }

  /// 固件更新流程
  pub fn firmware_update(&mut self, new_firmware: &[u8]) -> Result<(), &'static str> {
    self.print_message("Starting firmware update...\r\n");

    // 验证新固件
    if new_firmware.len() < core::mem::size_of::<FirmwareHeader>() {
      return Err("Invalid firmware size");
    }

    // 解析固件头部
    let header = unsafe { *(new_firmware.as_ptr() as *const FirmwareHeader) };

    if !header.is_valid() {
      return Err("Invalid firmware header");
    }

    // 回滚保护检查
    if header.version < self.security_config.rollback_version {
      return Err("Rollback protection violation");
    }

    // 验证签名
    let firmware_data = &new_firmware[core::mem::size_of::<FirmwareHeader>()..];
    if !self
      .signature_verifier
      .verify(firmware_data, &header.signature)
    {
      return Err("Signature verification failed");
    }

    // 写入新固件
    self
      .flash_manager
      .write_firmware(APPLICATION_START, new_firmware)?;

    // 更新回滚版本
    // 在实际应用中，这应该写入安全存储区域

    self.print_message("Firmware update completed successfully\r\n");

    Ok(())
  }

  /// 跳转到应用程序
  fn jump_to_application(&self, app_address: u32) {
    self.print_message("Preparing to jump to application\r\n");

    // 禁用中断
    cortex_m::interrupt::disable();

    // 实际的跳转实现
    unsafe {
      // 读取应用程序的栈指针和复位向量
      let app_stack_ptr = *(app_address as *const u32);
      let app_reset_vector = *((app_address + 4) as *const u32);

      // 设置栈指针
      cortex_m::register::msp::write(app_stack_ptr);

      // 跳转到应用程序
      let app_entry: fn() -> ! = core::mem::transmute(app_reset_vector);
      app_entry();
    }
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
    for i in 0..pos / 2 {
      buffer.swap(i, pos - 1 - i);
    }

    for i in 0..pos {
      nb::block!(self.serial.write(buffer[i])).ok();
    }
  }
}

/// 安全引导系统
pub struct SecureBootSystem {
  bootloader: SecureBootloader,
  led_pin:
    stm32f4xx_hal::gpio::gpioc::PC13<stm32f4xx_hal::gpio::Output<stm32f4xx_hal::gpio::PushPull>>,
  button_pin:
    stm32f4xx_hal::gpio::gpioa::PA0<stm32f4xx_hal::gpio::Input<stm32f4xx_hal::gpio::PullUp>>,
}

impl SecureBootSystem {
  pub fn new(
    bootloader: SecureBootloader,
    led_pin: stm32f4xx_hal::gpio::gpioc::PC13<
      stm32f4xx_hal::gpio::Output<stm32f4xx_hal::gpio::PushPull>,
    >,
    button_pin: stm32f4xx_hal::gpio::gpioa::PA0<
      stm32f4xx_hal::gpio::Input<stm32f4xx_hal::gpio::PullUp>,
    >,
  ) -> Self {
    Self {
      bootloader,
      led_pin,
      button_pin,
    }
  }

  /// 系统初始化
  pub fn init(&mut self) -> Result<(), &'static str> {
    self.led_pin.set_high();
    Ok(())
  }

  /// 运行引导加载器
  pub fn run(&mut self) -> ! {
    // 检查是否强制进入更新模式
    let force_update = self.button_pin.is_low();

    if force_update {
      self.led_pin.set_low();
      // 进入固件更新模式
      // 在实际应用中，这里会等待通过串口或其他接口接收新固件
      loop {
        // 等待固件更新
        cortex_m::asm::delay(SYSTEM_CLOCK_HZ / 10); // 100ms
      }
    } else {
      // 正常启动流程
      let boot_status = self.bootloader.secure_boot();

      match boot_status {
        BootStatus::Success => {
          // 成功启动，不应该到达这里
          self.led_pin.set_high();
        }
        _ => {
          // 启动失败，闪烁LED指示错误
          loop {
            self.led_pin.set_low();
            cortex_m::asm::delay(SYSTEM_CLOCK_HZ / 4); // 250ms
            self.led_pin.set_high();
            cortex_m::asm::delay(SYSTEM_CLOCK_HZ / 4); // 250ms
          }
        }
      }
    }

    // 不应该到达这里
    loop {
      cortex_m::asm::wfi();
    }
  }
}

#[entry]
fn main() -> ! {
  // 获取设备外设
  let dp = stm32::Peripherals::take().unwrap();

  // 配置时钟
  let rcc = dp.RCC.constrain();
  let clocks = rcc.cfgr.sysclk(SYSTEM_CLOCK_HZ.Hz()).freeze();

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
  )
  .unwrap();

  // 初始化Flash
  let flash = Flash::new(dp.FLASH);
  let flash_manager = FlashManager::new(flash);

  // 配置安全参数
  let mut security_config = SecurityConfig::new();
  // 在实际应用中，这些密钥应该从安全存储区域读取
  security_config.public_key = [0x12; 32]; // 示例公钥
  security_config.aes_key = [0x34; 32]; // 示例AES密钥
  security_config.rollback_version = 1;

  // 创建组件
  let signature_verifier = SignatureVerifier::new(security_config.public_key);
  let firmware_decryptor = FirmwareDecryptor::new(security_config.aes_key);

  // 创建引导加载器
  let bootloader = SecureBootloader::new(
    flash_manager,
    signature_verifier,
    firmware_decryptor,
    security_config,
    serial,
  );

  // 创建系统
  let mut system = SecureBootSystem::new(bootloader, led_pin, button_pin);

  // 初始化系统
  system.init().unwrap();

  // 运行引导加载器
  system.run()
}
