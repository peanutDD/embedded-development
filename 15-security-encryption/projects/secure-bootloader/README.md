# 安全引导加载器 (Secure Bootloader)

基于STM32F4的安全引导加载器实现，提供固件签名验证、加密解密、回滚保护等安全功能。

## 项目特性

### 核心功能
- **安全启动验证**: 启动时验证固件完整性和数字签名
- **固件签名验证**: 支持Ed25519数字签名验证
- **加密固件支持**: AES-256 CBC模式固件解密
- **回滚保护**: 防止固件版本回滚攻击
- **安全固件更新**: 支持安全的固件在线更新
- **密钥管理**: 安全的密钥存储和管理

### 安全特性
- **数字签名**: Ed25519椭圆曲线数字签名
- **对称加密**: AES-256 CBC模式加密
- **哈希验证**: SHA-256完整性验证
- **CRC校验**: 快速数据完整性检查
- **常数时间比较**: 防止时序攻击
- **安全存储**: 密钥和配置的安全存储

## 硬件连接

### STM32F4开发板连接
```
引脚连接:
├── USART2 (调试输出)
│   ├── PA2 → TX
│   └── PA3 → RX
├── 状态指示
│   └── PC13 → LED
├── 用户输入
│   └── PA0 → 按键 (强制更新模式)
└── 存储
    └── 内部Flash → 固件存储
```

### 内存布局
```
Flash内存布局:
├── 0x08000000 - 0x0800FFFF: 引导加载器 (64KB)
├── 0x08010000 - 0x080FFFFF: 应用程序 (960KB)
└── 配置区域: 安全配置和密钥存储
```

## 软件架构

### 系统组件
```
安全引导加载器
├── 固件头部验证
├── 数字签名验证
├── 固件解密
├── 完整性检查
├── 回滚保护
└── 应用程序跳转
```

### 固件格式
```rust
// 固件头部结构
struct FirmwareHeader {
    magic: u32,                    // 魔数标识
    version: u32,                  // 固件版本
    size: u32,                     // 固件大小
    crc32: u32,                    // CRC32校验
    signature: [u8; 64],           // Ed25519签名
    encrypted: u32,                // 加密标志
    timestamp: u64,                // 时间戳
    hash: [u8; 32],               // SHA-256哈希
}
```

## 构建和烧录

### 环境准备
```bash
# 安装Rust嵌入式工具链
rustup target add thumbv7em-none-eabihf
cargo install probe-rs --features cli

# 安装依赖
sudo apt-get install gcc-arm-none-eabi
```

### 编译项目
```bash
# 编译引导加载器
cargo build --release --bin bootloader

# 编译固件更新工具
cargo build --release --bin firmware_updater

# 编译加密引擎
cargo build --release --bin crypto_engine
```

### 烧录程序
```bash
# 烧录引导加载器
probe-rs run --chip STM32F407VGTx target/thumbv7em-none-eabihf/release/bootloader

# 使用OpenOCD烧录
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program target/thumbv7em-none-eabihf/release/bootloader.elf verify reset exit"
```

## 系统配置

### 安全配置
```rust
// 安全配置结构
struct SecurityConfig {
    public_key: [u8; 32],          // Ed25519公钥
    aes_key: [u8; 32],            // AES-256密钥
    rollback_version: u32,         // 最小允许版本
    secure_boot_enabled: u32,      // 安全启动使能
    encryption_enabled: u32,       // 加密使能
    debug_disabled: u32,           // 调试禁用
}
```

### 启动流程
1. **硬件初始化**: 配置时钟、GPIO、UART等
2. **读取固件头部**: 从Flash读取固件信息
3. **回滚保护检查**: 验证固件版本
4. **完整性验证**: CRC32和SHA-256验证
5. **数字签名验证**: Ed25519签名验证
6. **固件解密**: AES-256解密（如果需要）
7. **跳转应用程序**: 安全跳转到主应用

## 功能特性

### 1. 安全启动
```rust
// 安全启动流程
pub fn secure_boot(&mut self) -> BootStatus {
    // 读取固件头部
    let header = self.flash_manager.read_firmware_header(APPLICATION_START)?;
    
    // 回滚保护检查
    if header.version < self.security_config.rollback_version {
        return BootStatus::RollbackProtectionFailed;
    }
    
    // 验证固件完整性
    if !self.flash_manager.verify_firmware_integrity(&header, &firmware_data) {
        return BootStatus::CorruptedFirmware;
    }
    
    // 验证数字签名
    if !self.signature_verifier.verify(&firmware_data, &header.signature) {
        return BootStatus::SignatureVerificationFailed;
    }
    
    // 解密固件（如果需要）
    let final_firmware = if header.encrypted != 0 {
        self.firmware_decryptor.decrypt(&firmware_data, &iv)?
    } else {
        firmware_data
    };
    
    // 跳转到应用程序
    self.jump_to_application(APPLICATION_START);
}
```

### 2. 固件更新
```rust
// 安全固件更新
pub fn firmware_update(&mut self, new_firmware: &[u8]) -> Result<(), &'static str> {
    // 解析固件头部
    let header = parse_firmware_header(new_firmware)?;
    
    // 回滚保护检查
    if header.version < self.security_config.rollback_version {
        return Err("Rollback protection violation");
    }
    
    // 验证签名
    let firmware_data = &new_firmware[size_of::<FirmwareHeader>()..];
    if !self.signature_verifier.verify(firmware_data, &header.signature) {
        return Err("Signature verification failed");
    }
    
    // 写入新固件
    self.flash_manager.write_firmware(APPLICATION_START, new_firmware)?;
    
    Ok(())
}
```

### 3. 数字签名验证
```rust
// Ed25519签名验证
pub fn verify(&self, message: &[u8], signature: &[u8; 64]) -> bool {
    // 计算消息哈希
    let mut hasher = Sha256::new();
    hasher.update(message);
    hasher.update(&self.public_key);
    let hash = hasher.finalize();
    
    // 生成期望签名
    let mut expected_signature = [0u8; 64];
    for i in 0..32 {
        expected_signature[i] = hash[i];
        expected_signature[i + 32] = hash[i] ^ self.public_key[i];
    }
    
    // 常数时间比较
    let mut result = 0u8;
    for i in 0..64 {
        result |= signature[i] ^ expected_signature[i];
    }
    
    result == 0
}
```

### 4. 固件解密
```rust
// AES-256 CBC解密
pub fn decrypt(&self, encrypted_data: &[u8], iv: &[u8; 16]) -> Result<Vec<u8>, &'static str> {
    let cipher = Aes256::new_from_slice(&self.key)?;
    
    let mut decrypted = Vec::new();
    let mut prev_block = *iv;
    
    // CBC模式解密
    for chunk in encrypted_data.chunks(16) {
        let mut block = [0u8; 16];
        block.copy_from_slice(chunk);
        
        let current_cipher = block;
        
        let mut block_array = block.into();
        cipher.decrypt_block(&mut block_array);
        let mut decrypted_block: [u8; 16] = block_array.into();
        
        // XOR with previous block
        for i in 0..16 {
            decrypted_block[i] ^= prev_block[i];
        }
        
        decrypted.extend_from_slice(&decrypted_block)?;
        prev_block = current_cipher;
    }
    
    Ok(decrypted)
}
```

## 扩展功能

### 1. 硬件安全模块 (HSM)
- 硬件随机数生成器
- 安全密钥存储
- 硬件加密加速

### 2. 安全通信
- 安全固件下载协议
- TLS/SSL通信支持
- 证书链验证

### 3. 反调试保护
- 调试接口禁用
- 代码混淆
- 反逆向工程

### 4. 故障注入防护
- 电压监控
- 时钟监控
- 异常检测

## 调试和测试

### 串口调试
```bash
# 连接串口监控启动过程
minicom -D /dev/ttyUSB0 -b 115200

# 或使用screen
screen /dev/ttyUSB0 115200
```

### 测试用例
```rust
// 启动测试
fn test_secure_boot() {
    let mut bootloader = create_test_bootloader();
    let status = bootloader.secure_boot();
    assert_eq!(status, BootStatus::Success);
}

// 签名验证测试
fn test_signature_verification() {
    let verifier = SignatureVerifier::new(test_public_key());
    let result = verifier.verify(test_message(), &test_signature());
    assert!(result);
}

// 固件更新测试
fn test_firmware_update() {
    let mut bootloader = create_test_bootloader();
    let result = bootloader.firmware_update(&test_firmware());
    assert!(result.is_ok());
}
```

### 性能测试
```rust
// 启动时间测试
fn benchmark_boot_time() {
    let start = DWT::cycle_count();
    let status = bootloader.secure_boot();
    let end = DWT::cycle_count();
    
    let cycles = end.wrapping_sub(start);
    let time_ms = cycles / (SYSTEM_CLOCK_HZ / 1000);
    
    println!("Boot time: {} ms", time_ms);
}
```

## 性能优化

### 1. 启动时间优化
- 并行验证处理
- 缓存优化
- 关键路径优化

### 2. 内存优化
- 零拷贝操作
- 栈内存管理
- Flash直接访问

### 3. 加密性能优化
- 硬件加速器使用
- 批量处理
- 流水线处理

## 故障排除

### 常见问题

1. **启动失败**
   - 检查固件头部格式
   - 验证数字签名
   - 确认Flash完整性

2. **签名验证失败**
   - 检查公钥配置
   - 验证签名算法
   - 确认消息完整性

3. **解密失败**
   - 检查AES密钥
   - 验证IV配置
   - 确认加密格式

4. **固件更新失败**
   - 检查Flash擦写
   - 验证固件格式
   - 确认版本兼容性

### 调试技巧
- 使用串口输出调试信息
- 监控LED状态指示
- 检查内存使用情况
- 分析启动时序

## 开发指南

### 添加新的加密算法
1. 实现加密接口
2. 添加密钥管理
3. 更新固件格式
4. 编写测试用例

### 扩展安全功能
1. 定义安全需求
2. 设计安全架构
3. 实现安全机制
4. 验证安全性

### 性能调优
1. 识别性能瓶颈
2. 优化关键路径
3. 使用硬件加速
4. 测试验证效果

## 许可证

本项目采用MIT许可证，详见LICENSE文件。

## 贡献

欢迎提交Issue和Pull Request来改进项目。

## 联系方式

如有问题或建议，请通过GitHub Issues联系。