# 固件更新 (Firmware Updates)

嵌入式系统的固件更新机制，包括引导程序设计、OTA更新、安全验证和回滚机制。

## 1. 固件更新概述

### 1.1 更新方式分类
- **有线更新**: 通过UART、USB、SWD等接口
- **无线更新 (OTA)**: 通过WiFi、蓝牙、LoRa等无线接口
- **存储介质更新**: 通过SD卡、USB存储等外部存储

### 1.2 系统架构
```
Flash Memory Layout:
┌─────────────────┐ 0x08000000
│   Bootloader    │ (32KB)
├─────────────────┤ 0x08008000
│   App Slot A    │ (240KB)
├─────────────────┤ 0x08044000
│   App Slot B    │ (240KB)
├─────────────────┤ 0x08080000
│   Config/Data   │ (Remaining)
└─────────────────┘
```

## 2. 引导程序 (Bootloader) 设计

### 2.1 引导程序结构
```rust
#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use stm32f4xx_hal::{prelude::*, stm32, flash::Flash};

// 内存布局定义
const BOOTLOADER_SIZE: u32 = 0x8000;      // 32KB
const APP_SLOT_A: u32 = 0x08008000;       // 应用槽A
const APP_SLOT_B: u32 = 0x08044000;       // 应用槽B
const APP_SLOT_SIZE: u32 = 0x3C000;       // 240KB per slot

// 固件头部结构
#[repr(C)]
#[derive(Clone, Copy)]
struct FirmwareHeader {
    magic: u32,           // 魔数标识
    version: u32,         // 版本号
    size: u32,            // 固件大小
    crc32: u32,           // CRC32校验
    timestamp: u32,       // 时间戳
    flags: u32,           // 标志位
}

impl FirmwareHeader {
    const MAGIC: u32 = 0xDEADBEEF;
    
    fn is_valid(&self) -> bool {
        self.magic == Self::MAGIC && self.size <= APP_SLOT_SIZE
    }
    
    fn verify_crc(&self, data: &[u8]) -> bool {
        let calculated_crc = crc32(data);
        calculated_crc == self.crc32
    }
}

// 引导配置
#[repr(C)]
struct BootConfig {
    active_slot: u8,      // 当前活动槽 (0=A, 1=B)
    update_pending: u8,   // 更新待处理标志
    boot_attempts: u8,    // 启动尝试次数
    max_attempts: u8,     // 最大尝试次数
    fallback_slot: u8,    // 回退槽
    reserved: [u8; 3],    // 保留字节
}

impl BootConfig {
    fn new() -> Self {
        Self {
            active_slot: 0,
            update_pending: 0,
            boot_attempts: 0,
            max_attempts: 3,
            fallback_slot: 0,
            reserved: [0; 3],
        }
    }
}

// 引导程序主逻辑
struct Bootloader {
    flash: Flash,
    config: BootConfig,
}

impl Bootloader {
    fn new(flash: Flash) -> Self {
        let config = Self::load_config().unwrap_or_else(|| BootConfig::new());
        Self { flash, config }
    }
    
    fn boot(&mut self) -> ! {
        // 检查是否有待处理的更新
        if self.config.update_pending != 0 {
            self.handle_update();
        }
        
        // 尝试启动应用程序
        match self.try_boot_application() {
            Ok(_) => {
                // 启动成功，重置尝试计数
                self.config.boot_attempts = 0;
                self.save_config();
                self.jump_to_application();
            }
            Err(_) => {
                // 启动失败，增加尝试计数
                self.config.boot_attempts += 1;
                
                if self.config.boot_attempts >= self.config.max_attempts {
                    // 超过最大尝试次数，切换到备用槽
                    self.fallback_to_backup();
                }
                
                self.save_config();
                self.system_reset();
            }
        }
    }
    
    fn try_boot_application(&self) -> Result<(), BootError> {
        let slot_address = if self.config.active_slot == 0 {
            APP_SLOT_A
        } else {
            APP_SLOT_B
        };
        
        // 验证固件头部
        let header = self.read_firmware_header(slot_address)?;
        if !header.is_valid() {
            return Err(BootError::InvalidHeader);
        }
        
        // 验证固件完整性
        let firmware_data = self.read_firmware_data(slot_address, header.size)?;
        if !header.verify_crc(&firmware_data) {
            return Err(BootError::CrcMismatch);
        }
        
        Ok(())
    }
    
    fn handle_update(&mut self) {
        let source_slot = if self.config.active_slot == 0 { 1 } else { 0 };
        let target_slot = self.config.active_slot;
        
        match self.validate_and_install_update(source_slot, target_slot) {
            Ok(_) => {
                // 更新成功，切换活动槽
                self.config.active_slot = source_slot;
                self.config.update_pending = 0;
                self.config.boot_attempts = 0;
            }
            Err(_) => {
                // 更新失败，清除待处理标志
                self.config.update_pending = 0;
            }
        }
        
        self.save_config();
    }
    
    fn validate_and_install_update(&mut self, source_slot: u8, _target_slot: u8) -> Result<(), BootError> {
        let source_address = if source_slot == 0 { APP_SLOT_A } else { APP_SLOT_B };
        
        // 读取并验证新固件
        let header = self.read_firmware_header(source_address)?;
        if !header.is_valid() {
            return Err(BootError::InvalidHeader);
        }
        
        let firmware_data = self.read_firmware_data(source_address, header.size)?;
        if !header.verify_crc(&firmware_data) {
            return Err(BootError::CrcMismatch);
        }
        
        // 验证固件签名 (如果启用)
        #[cfg(feature = "secure-boot")]
        self.verify_signature(&header, &firmware_data)?;
        
        Ok(())
    }
    
    fn fallback_to_backup(&mut self) {
        // 切换到备用槽
        self.config.active_slot = self.config.fallback_slot;
        self.config.boot_attempts = 0;
        
        // 记录回退事件
        self.log_fallback_event();
    }
    
    fn jump_to_application(&self) -> ! {
        let app_address = if self.config.active_slot == 0 {
            APP_SLOT_A
        } else {
            APP_SLOT_B
        };
        
        unsafe {
            // 读取应用程序的栈指针和入口点
            let stack_ptr = core::ptr::read_volatile(app_address as *const u32);
            let entry_point = core::ptr::read_volatile((app_address + 4) as *const u32);
            
            // 设置栈指针
            cortex_m::register::msp::write(stack_ptr);
            
            // 跳转到应用程序
            let app_entry: fn() -> ! = core::mem::transmute(entry_point);
            app_entry();
        }
    }
    
    fn read_firmware_header(&self, address: u32) -> Result<FirmwareHeader, BootError> {
        unsafe {
            let header_ptr = address as *const FirmwareHeader;
            Ok(core::ptr::read_volatile(header_ptr))
        }
    }
    
    fn read_firmware_data(&self, address: u32, size: u32) -> Result<&[u8], BootError> {
        unsafe {
            let data_ptr = (address + core::mem::size_of::<FirmwareHeader>() as u32) as *const u8;
            Ok(core::slice::from_raw_parts(data_ptr, size as usize))
        }
    }
    
    fn load_config() -> Option<BootConfig> {
        // 从Flash的配置区域加载配置
        // 实现省略
        None
    }
    
    fn save_config(&self) {
        // 保存配置到Flash
        // 实现省略
    }
    
    fn log_fallback_event(&self) {
        // 记录回退事件到日志
        // 实现省略
    }
    
    fn system_reset(&self) -> ! {
        cortex_m::peripheral::SCB::sys_reset();
    }
}

#[derive(Debug)]
enum BootError {
    InvalidHeader,
    CrcMismatch,
    SignatureError,
    FlashError,
}

// CRC32计算
fn crc32(data: &[u8]) -> u32 {
    let mut crc = 0xFFFFFFFF;
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

#[entry]
fn main() -> ! {
    // 初始化硬件
    let dp = stm32::Peripherals::take().unwrap();
    let flash = Flash::new(dp.FLASH);
    
    // 创建并启动引导程序
    let mut bootloader = Bootloader::new(flash);
    bootloader.boot();
}
```

### 2.2 安全引导 (Secure Boot)
```rust
#[cfg(feature = "secure-boot")]
mod secure_boot {
    use super::*;
    use sha2::{Sha256, Digest};
    use rsa::{RsaPublicKey, PaddingScheme, PublicKey};
    
    // 公钥存储 (嵌入到引导程序中)
    const PUBLIC_KEY: &[u8] = include_bytes!("public_key.der");
    
    // 签名验证
    pub fn verify_signature(header: &FirmwareHeader, data: &[u8]) -> Result<(), BootError> {
        // 计算固件哈希
        let mut hasher = Sha256::new();
        hasher.update(data);
        let hash = hasher.finalize();
        
        // 从固件头部提取签名
        let signature = extract_signature(header)?;
        
        // 加载公钥
        let public_key = load_public_key()?;
        
        // 验证签名
        match public_key.verify(PaddingScheme::PKCS1v15Sign, Some(&hash), &signature) {
            Ok(_) => Ok(()),
            Err(_) => Err(BootError::SignatureError),
        }
    }
    
    fn extract_signature(header: &FirmwareHeader) -> Result<Vec<u8>, BootError> {
        // 从固件头部或固件末尾提取签名
        // 实现省略
        Ok(vec![])
    }
    
    fn load_public_key() -> Result<RsaPublicKey, BootError> {
        // 加载嵌入的公钥
        // 实现省略
        Err(BootError::SignatureError)
    }
}
```

## 3. OTA更新实现

### 3.1 OTA更新管理器
```rust
use heapless::{Vec, String};
use embedded_hal::digital::v2::OutputPin;
use nb::block;

// OTA更新状态
#[derive(Debug, Clone, Copy)]
enum OtaState {
    Idle,
    Downloading,
    Verifying,
    Installing,
    Complete,
    Error(OtaError),
}

#[derive(Debug, Clone, Copy)]
enum OtaError {
    NetworkError,
    InvalidFirmware,
    InsufficientSpace,
    VerificationFailed,
    InstallationFailed,
}

// OTA更新管理器
struct OtaManager<WIFI, LED> {
    wifi: WIFI,
    status_led: LED,
    state: OtaState,
    download_buffer: Vec<u8, 4096>,
    total_size: u32,
    downloaded_size: u32,
    current_slot: u8,
}

impl<WIFI, LED> OtaManager<WIFI, LED>
where
    WIFI: WifiInterface,
    LED: OutputPin,
{
    fn new(wifi: WIFI, status_led: LED) -> Self {
        Self {
            wifi,
            status_led,
            state: OtaState::Idle,
            download_buffer: Vec::new(),
            total_size: 0,
            downloaded_size: 0,
            current_slot: 0,
        }
    }
    
    fn check_for_updates(&mut self) -> Result<Option<FirmwareInfo>, OtaError> {
        // 连接到更新服务器
        self.wifi.connect_to_server("update.example.com", 443)?;
        
        // 发送版本查询请求
        let current_version = get_current_version();
        let request = format!(
            "GET /api/firmware/check?version={}&device_id={} HTTP/1.1\r\n\
             Host: update.example.com\r\n\
             Connection: close\r\n\r\n",
            current_version,
            get_device_id()
        );
        
        self.wifi.send(request.as_bytes())?;
        
        // 接收响应
        let response = self.wifi.receive()?;
        let firmware_info = parse_update_response(&response)?;
        
        Ok(firmware_info)
    }
    
    fn download_firmware(&mut self, firmware_info: &FirmwareInfo) -> Result<(), OtaError> {
        self.state = OtaState::Downloading;
        self.total_size = firmware_info.size;
        self.downloaded_size = 0;
        
        // 确定下载到哪个槽
        self.current_slot = get_inactive_slot();
        let slot_address = get_slot_address(self.current_slot);
        
        // 擦除目标槽
        erase_flash_sector(slot_address, firmware_info.size)?;
        
        // 下载固件
        let mut offset = 0;
        while offset < firmware_info.size {
            // 请求数据块
            let chunk_size = (firmware_info.size - offset).min(self.download_buffer.capacity() as u32);
            let chunk = self.download_chunk(firmware_info, offset, chunk_size)?;
            
            // 写入Flash
            write_flash(slot_address + offset, &chunk)?;
            
            offset += chunk.len() as u32;
            self.downloaded_size = offset;
            
            // 更新进度指示
            self.update_progress_indicator();
        }
        
        Ok(())
    }
    
    fn download_chunk(&mut self, firmware_info: &FirmwareInfo, offset: u32, size: u32) -> Result<Vec<u8, 4096>, OtaError> {
        // 发送HTTP Range请求
        let request = format!(
            "GET {} HTTP/1.1\r\n\
             Host: update.example.com\r\n\
             Range: bytes={}-{}\r\n\
             Connection: keep-alive\r\n\r\n",
            firmware_info.download_url,
            offset,
            offset + size - 1
        );
        
        self.wifi.send(request.as_bytes())?;
        
        // 接收数据块
        let response = self.wifi.receive()?;
        let chunk_data = extract_chunk_data(&response)?;
        
        Ok(chunk_data)
    }
    
    fn verify_firmware(&mut self) -> Result<(), OtaError> {
        self.state = OtaState::Verifying;
        
        let slot_address = get_slot_address(self.current_slot);
        
        // 读取固件头部
        let header = read_firmware_header(slot_address)?;
        if !header.is_valid() {
            return Err(OtaError::InvalidFirmware);
        }
        
        // 验证CRC
        let firmware_data = read_firmware_data(slot_address, header.size)?;
        if !header.verify_crc(&firmware_data) {
            return Err(OtaError::VerificationFailed);
        }
        
        // 验证签名 (如果启用)
        #[cfg(feature = "secure-boot")]
        verify_firmware_signature(&header, &firmware_data)?;
        
        Ok(())
    }
    
    fn install_firmware(&mut self) -> Result<(), OtaError> {
        self.state = OtaState::Installing;
        
        // 设置更新标志
        set_update_pending_flag(self.current_slot)?;
        
        // 触发重启以应用更新
        self.state = OtaState::Complete;
        system_reset();
    }
    
    fn update_progress_indicator(&mut self) {
        let progress = (self.downloaded_size * 100) / self.total_size;
        
        // 通过LED闪烁显示进度
        match progress {
            0..=25 => self.blink_led(1),
            26..=50 => self.blink_led(2),
            51..=75 => self.blink_led(3),
            76..=100 => self.blink_led(4),
        }
    }
    
    fn blink_led(&mut self, count: u8) {
        for _ in 0..count {
            self.status_led.set_high().ok();
            delay_ms(100);
            self.status_led.set_low().ok();
            delay_ms(100);
        }
    }
    
    fn handle_error(&mut self, error: OtaError) {
        self.state = OtaState::Error(error);
        
        // 错误指示 (快速闪烁)
        for _ in 0..10 {
            self.status_led.set_high().ok();
            delay_ms(50);
            self.status_led.set_low().ok();
            delay_ms(50);
        }
    }
}

// 固件信息结构
#[derive(Debug)]
struct FirmwareInfo {
    version: String<32>,
    size: u32,
    checksum: String<64>,
    download_url: String<128>,
    release_notes: String<256>,
}

// WiFi接口抽象
trait WifiInterface {
    fn connect_to_server(&mut self, host: &str, port: u16) -> Result<(), OtaError>;
    fn send(&mut self, data: &[u8]) -> Result<(), OtaError>;
    fn receive(&mut self) -> Result<Vec<u8, 2048>, OtaError>;
}

// 辅助函数
fn get_current_version() -> u32 {
    // 获取当前固件版本
    0x010000 // 1.0.0
}

fn get_device_id() -> String<32> {
    // 获取设备唯一ID
    let mut id = String::new();
    id.push_str("DEV123456").ok();
    id
}

fn get_inactive_slot() -> u8 {
    // 获取非活动槽编号
    let active_slot = get_active_slot();
    if active_slot == 0 { 1 } else { 0 }
}

fn get_active_slot() -> u8 {
    // 从引导配置获取活动槽
    0
}

fn get_slot_address(slot: u8) -> u32 {
    if slot == 0 { APP_SLOT_A } else { APP_SLOT_B }
}

fn parse_update_response(response: &[u8]) -> Result<Option<FirmwareInfo>, OtaError> {
    // 解析HTTP响应，提取固件信息
    // 实现省略
    Ok(None)
}

fn extract_chunk_data(response: &[u8]) -> Result<Vec<u8, 4096>, OtaError> {
    // 从HTTP响应中提取数据块
    // 实现省略
    Ok(Vec::new())
}

fn erase_flash_sector(address: u32, size: u32) -> Result<(), OtaError> {
    // 擦除Flash扇区
    // 实现省略
    Ok(())
}

fn write_flash(address: u32, data: &[u8]) -> Result<(), OtaError> {
    // 写入Flash
    // 实现省略
    Ok(())
}

fn read_firmware_header(address: u32) -> Result<FirmwareHeader, OtaError> {
    // 读取固件头部
    // 实现省略
    Ok(FirmwareHeader {
        magic: 0,
        version: 0,
        size: 0,
        crc32: 0,
        timestamp: 0,
        flags: 0,
    })
}

fn read_firmware_data(address: u32, size: u32) -> Result<&'static [u8], OtaError> {
    // 读取固件数据
    // 实现省略
    Ok(&[])
}

fn set_update_pending_flag(slot: u8) -> Result<(), OtaError> {
    // 设置更新待处理标志
    // 实现省略
    Ok(())
}

fn system_reset() -> ! {
    cortex_m::peripheral::SCB::sys_reset();
}

fn delay_ms(ms: u32) {
    // 延时实现
    // 实现省略
}
```

### 3.2 增量更新 (Delta Updates)
```rust
// 增量更新管理
struct DeltaUpdateManager {
    base_version: u32,
    target_version: u32,
    patch_buffer: Vec<u8, 2048>,
}

impl DeltaUpdateManager {
    fn apply_delta_patch(&mut self, patch_data: &[u8]) -> Result<(), OtaError> {
        // 解析补丁头部
        let patch_header = self.parse_patch_header(patch_data)?;
        
        // 验证基础版本
        if patch_header.base_version != self.base_version {
            return Err(OtaError::InvalidFirmware);
        }
        
        // 应用二进制差分补丁
        self.apply_binary_diff(&patch_header, patch_data)?;
        
        Ok(())
    }
    
    fn parse_patch_header(&self, patch_data: &[u8]) -> Result<PatchHeader, OtaError> {
        // 解析补丁头部
        // 实现省略
        Ok(PatchHeader {
            base_version: 0,
            target_version: 0,
            patch_size: 0,
            checksum: 0,
        })
    }
    
    fn apply_binary_diff(&mut self, header: &PatchHeader, patch_data: &[u8]) -> Result<(), OtaError> {
        // 应用二进制差分算法 (如bsdiff)
        // 实现省略
        Ok(())
    }
}

#[derive(Debug)]
struct PatchHeader {
    base_version: u32,
    target_version: u32,
    patch_size: u32,
    checksum: u32,
}
```

## 4. 回滚机制

### 4.1 自动回滚
```rust
// 应用程序健康检查
struct HealthChecker {
    boot_time: u32,
    last_heartbeat: u32,
    error_count: u32,
    max_errors: u32,
}

impl HealthChecker {
    fn new() -> Self {
        Self {
            boot_time: get_system_time(),
            last_heartbeat: 0,
            error_count: 0,
            max_errors: 5,
        }
    }
    
    fn send_heartbeat(&mut self) {
        self.last_heartbeat = get_system_time();
        
        // 向引导程序发送心跳信号
        self.notify_bootloader_healthy();
    }
    
    fn report_error(&mut self) {
        self.error_count += 1;
        
        if self.error_count >= self.max_errors {
            // 触发回滚
            self.trigger_rollback();
        }
    }
    
    fn check_boot_timeout(&self) -> bool {
        let elapsed = get_system_time() - self.boot_time;
        elapsed > 30000 // 30秒启动超时
    }
    
    fn notify_bootloader_healthy(&self) {
        // 通过特定内存位置或寄存器通知引导程序
        unsafe {
            core::ptr::write_volatile(0x20000000 as *mut u32, 0xDEADBEEF);
        }
    }
    
    fn trigger_rollback(&self) {
        // 设置回滚标志并重启
        set_rollback_flag();
        system_reset();
    }
}

fn set_rollback_flag() {
    // 设置回滚标志到持久存储
    // 实现省略
}

fn get_system_time() -> u32 {
    // 获取系统时间 (毫秒)
    // 实现省略
    0
}
```

### 4.2 用户触发回滚
```rust
// 用户接口回滚
struct UserRollbackInterface<BUTTON> {
    button: BUTTON,
    press_start_time: Option<u32>,
    long_press_duration: u32,
}

impl<BUTTON> UserRollbackInterface<BUTTON>
where
    BUTTON: embedded_hal::digital::v2::InputPin,
{
    fn new(button: BUTTON) -> Self {
        Self {
            button,
            press_start_time: None,
            long_press_duration: 5000, // 5秒长按
        }
    }
    
    fn check_rollback_trigger(&mut self) -> bool {
        let button_pressed = self.button.is_low().unwrap_or(false);
        let current_time = get_system_time();
        
        match (button_pressed, self.press_start_time) {
            (true, None) => {
                // 按钮刚被按下
                self.press_start_time = Some(current_time);
                false
            }
            (true, Some(start_time)) => {
                // 按钮持续按下
                let press_duration = current_time - start_time;
                if press_duration >= self.long_press_duration {
                    // 长按触发回滚
                    self.press_start_time = None;
                    true
                } else {
                    false
                }
            }
            (false, _) => {
                // 按钮释放
                self.press_start_time = None;
                false
            }
        }
    }
}
```

## 5. 固件签名和验证

### 5.1 固件签名工具
```python
#!/usr/bin/env python3
"""
固件签名工具
"""

import hashlib
import struct
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import rsa, padding

class FirmwareSigner:
    def __init__(self, private_key_path):
        with open(private_key_path, 'rb') as f:
            self.private_key = serialization.load_pem_private_key(
                f.read(), password=None
            )
    
    def sign_firmware(self, firmware_path, output_path):
        # 读取固件文件
        with open(firmware_path, 'rb') as f:
            firmware_data = f.read()
        
        # 计算哈希
        digest = hashlib.sha256(firmware_data).digest()
        
        # 生成签名
        signature = self.private_key.sign(
            digest,
            padding.PSS(
                mgf=padding.MGF1(hashes.SHA256()),
                salt_length=padding.PSS.MAX_LENGTH
            ),
            hashes.SHA256()
        )
        
        # 创建固件头部
        header = self.create_firmware_header(firmware_data, signature)
        
        # 写入签名固件
        with open(output_path, 'wb') as f:
            f.write(header)
            f.write(firmware_data)
            f.write(signature)
        
        print(f"Firmware signed successfully: {output_path}")
    
    def create_firmware_header(self, firmware_data, signature):
        magic = 0xDEADBEEF
        version = 0x010001  # 1.0.1
        size = len(firmware_data)
        crc32 = self.calculate_crc32(firmware_data)
        timestamp = int(time.time())
        flags = 0x01  # 签名标志
        
        return struct.pack('<LLLLLL', magic, version, size, crc32, timestamp, flags)
    
    def calculate_crc32(self, data):
        import zlib
        return zlib.crc32(data) & 0xffffffff

if __name__ == "__main__":
    import sys
    import time
    
    if len(sys.argv) != 4:
        print("Usage: sign_firmware.py <private_key> <firmware> <output>")
        sys.exit(1)
    
    signer = FirmwareSigner(sys.argv[1])
    signer.sign_firmware(sys.argv[2], sys.argv[3])
```

### 5.2 密钥管理
```bash
#!/bin/bash
# 生成RSA密钥对

# 生成私钥
openssl genrsa -out private_key.pem 2048

# 生成公钥
openssl rsa -in private_key.pem -pubout -out public_key.pem

# 转换公钥为DER格式 (嵌入到固件中)
openssl rsa -in private_key.pem -pubout -outform DER -out public_key.der

echo "Key pair generated successfully"
echo "Private key: private_key.pem (keep secure!)"
echo "Public key: public_key.pem"
echo "Public key (DER): public_key.der (embed in bootloader)"
```

## 6. 更新服务器

### 6.1 简单HTTP更新服务器
```python
#!/usr/bin/env python3
"""
简单的固件更新服务器
"""

from flask import Flask, request, jsonify, send_file
import os
import json
import hashlib

app = Flask(__name__)

# 固件存储目录
FIRMWARE_DIR = "firmware_releases"

class FirmwareManager:
    def __init__(self):
        self.firmware_db = self.load_firmware_database()
    
    def load_firmware_database(self):
        db_path = os.path.join(FIRMWARE_DIR, "firmware_db.json")
        if os.path.exists(db_path):
            with open(db_path, 'r') as f:
                return json.load(f)
        return {}
    
    def save_firmware_database(self):
        db_path = os.path.join(FIRMWARE_DIR, "firmware_db.json")
        with open(db_path, 'w') as f:
            json.dump(self.firmware_db, f, indent=2)
    
    def get_latest_version(self, device_type):
        if device_type in self.firmware_db:
            versions = self.firmware_db[device_type]
            return max(versions.keys(), key=lambda x: tuple(map(int, x.split('.'))))
        return None
    
    def get_firmware_info(self, device_type, version):
        if device_type in self.firmware_db:
            return self.firmware_db[device_type].get(version)
        return None

firmware_manager = FirmwareManager()

@app.route('/api/firmware/check')
def check_for_updates():
    device_type = request.args.get('device_type', 'default')
    current_version = request.args.get('version', '0.0.0')
    device_id = request.args.get('device_id', 'unknown')
    
    # 获取最新版本
    latest_version = firmware_manager.get_latest_version(device_type)
    
    if latest_version and latest_version > current_version:
        firmware_info = firmware_manager.get_firmware_info(device_type, latest_version)
        return jsonify({
            'update_available': True,
            'version': latest_version,
            'size': firmware_info['size'],
            'checksum': firmware_info['checksum'],
            'download_url': f'/api/firmware/download/{device_type}/{latest_version}',
            'release_notes': firmware_info.get('release_notes', '')
        })
    else:
        return jsonify({'update_available': False})

@app.route('/api/firmware/download/<device_type>/<version>')
def download_firmware(device_type, version):
    firmware_info = firmware_manager.get_firmware_info(device_type, version)
    if firmware_info:
        firmware_path = os.path.join(FIRMWARE_DIR, firmware_info['filename'])
        if os.path.exists(firmware_path):
            return send_file(firmware_path, as_attachment=True)
    
    return jsonify({'error': 'Firmware not found'}), 404

@app.route('/api/firmware/upload', methods=['POST'])
def upload_firmware():
    if 'firmware' not in request.files:
        return jsonify({'error': 'No firmware file'}), 400
    
    file = request.files['firmware']
    device_type = request.form.get('device_type', 'default')
    version = request.form.get('version')
    release_notes = request.form.get('release_notes', '')
    
    if not version:
        return jsonify({'error': 'Version required'}), 400
    
    # 保存固件文件
    filename = f"{device_type}_{version}.bin"
    filepath = os.path.join(FIRMWARE_DIR, filename)
    file.save(filepath)
    
    # 计算校验和
    with open(filepath, 'rb') as f:
        checksum = hashlib.sha256(f.read()).hexdigest()
    
    # 更新数据库
    if device_type not in firmware_manager.firmware_db:
        firmware_manager.firmware_db[device_type] = {}
    
    firmware_manager.firmware_db[device_type][version] = {
        'filename': filename,
        'size': os.path.getsize(filepath),
        'checksum': checksum,
        'release_notes': release_notes,
        'upload_time': time.time()
    }
    
    firmware_manager.save_firmware_database()
    
    return jsonify({'message': 'Firmware uploaded successfully'})

if __name__ == '__main__':
    os.makedirs(FIRMWARE_DIR, exist_ok=True)
    app.run(host='0.0.0.0', port=8080, debug=True)
```

## 7. 测试和验证

### 7.1 更新流程测试
```rust
#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_firmware_header_validation() {
        let valid_header = FirmwareHeader {
            magic: FirmwareHeader::MAGIC,
            version: 0x010001,
            size: 1024,
            crc32: 0x12345678,
            timestamp: 1234567890,
            flags: 0,
        };
        
        assert!(valid_header.is_valid());
        
        let invalid_header = FirmwareHeader {
            magic: 0x00000000,
            version: 0x010001,
            size: 1024,
            crc32: 0x12345678,
            timestamp: 1234567890,
            flags: 0,
        };
        
        assert!(!invalid_header.is_valid());
    }
    
    #[test]
    fn test_crc32_calculation() {
        let data = b"Hello, World!";
        let expected_crc = 0xebe6c6e6;
        let calculated_crc = crc32(data);
        
        assert_eq!(calculated_crc, expected_crc);
    }
    
    #[test]
    fn test_boot_config_serialization() {
        let config = BootConfig::new();
        
        // 测试配置序列化和反序列化
        // 实现省略
    }
}
```

### 7.2 集成测试脚本
```bash
#!/bin/bash
# 固件更新集成测试

set -e

echo "Starting firmware update integration test..."

# 构建测试固件
echo "Building test firmware..."
cargo build --release --bin test_firmware

# 签名固件
echo "Signing firmware..."
python3 tools/sign_firmware.py keys/private_key.pem \
    target/thumbv7em-none-eabihf/release/test_firmware \
    test_firmware_signed.bin

# 启动测试服务器
echo "Starting test server..."
python3 tools/update_server.py &
SERVER_PID=$!

# 等待服务器启动
sleep 2

# 上传测试固件
echo "Uploading test firmware..."
curl -X POST -F "firmware=@test_firmware_signed.bin" \
     -F "device_type=test_device" \
     -F "version=1.0.1" \
     -F "release_notes=Test firmware" \
     http://localhost:8080/api/firmware/upload

# 测试更新检查
echo "Testing update check..."
curl "http://localhost:8080/api/firmware/check?device_type=test_device&version=1.0.0"

# 清理
echo "Cleaning up..."
kill $SERVER_PID
rm -f test_firmware_signed.bin

echo "Integration test completed successfully!"
```

## 总结

固件更新是嵌入式系统的关键功能，需要考虑：

1. **安全性**: 签名验证、安全引导、防回滚攻击
2. **可靠性**: 双槽机制、自动回滚、健康检查
3. **效率**: 增量更新、断点续传、压缩传输
4. **用户体验**: 进度指示、错误处理、简单操作

关键设计原则：
- 永远保持一个可工作的固件版本
- 实现完整的验证和回滚机制
- 提供清晰的状态反馈
- 考虑网络中断和电源故障等异常情况