# 密码学基础

在嵌入式系统中，安全性是一个至关重要的考虑因素。本章将介绍嵌入式系统中常用的密码学算法和安全实现。

## 密码学概述

### 密码学的基本概念

密码学是研究如何在敌对环境中进行安全通信的学科。在嵌入式系统中，我们需要考虑：

- **机密性** - 确保数据不被未授权访问
- **完整性** - 确保数据未被篡改
- **身份验证** - 确认通信方的身份
- **不可否认性** - 防止发送方否认已发送的消息

### 对称加密

对称加密使用相同的密钥进行加密和解密，适合资源受限的嵌入式系统。

#### AES加密实现

```rust
use aes::Aes128;
use aes::cipher::{
    BlockCipher, BlockEncrypt, BlockDecrypt, KeyInit,
    generic_array::GenericArray,
};

/// AES-128加密器
pub struct AesEncryptor {
    cipher: Aes128,
}

impl AesEncryptor {
    /// 创建新的AES加密器
    pub fn new(key: &[u8; 16]) -> Self {
        let key = GenericArray::from_slice(key);
        let cipher = Aes128::new(key);
        
        Self { cipher }
    }
    
    /// 加密单个数据块
    pub fn encrypt_block(&self, block: &mut [u8; 16]) {
        let mut block = GenericArray::from_mut_slice(block);
        self.cipher.encrypt_block(&mut block);
    }
    
    /// 解密单个数据块
    pub fn decrypt_block(&self, block: &mut [u8; 16]) {
        let mut block = GenericArray::from_mut_slice(block);
        self.cipher.decrypt_block(&mut block);
    }
    
    /// ECB模式加密
    pub fn encrypt_ecb(&self, data: &mut [u8]) -> Result<(), &'static str> {
        if data.len() % 16 != 0 {
            return Err("Data length must be multiple of 16");
        }
        
        for chunk in data.chunks_exact_mut(16) {
            let mut block = [0u8; 16];
            block.copy_from_slice(chunk);
            self.encrypt_block(&mut block);
            chunk.copy_from_slice(&block);
        }
        
        Ok(())
    }
    
    /// ECB模式解密
    pub fn decrypt_ecb(&self, data: &mut [u8]) -> Result<(), &'static str> {
        if data.len() % 16 != 0 {
            return Err("Data length must be multiple of 16");
        }
        
        for chunk in data.chunks_exact_mut(16) {
            let mut block = [0u8; 16];
            block.copy_from_slice(chunk);
            self.decrypt_block(&mut block);
            chunk.copy_from_slice(&block);
        }
        
        Ok(())
    }
}

/// CBC模式加密器
pub struct AesCbcEncryptor {
    cipher: Aes128,
    iv: [u8; 16],
}

impl AesCbcEncryptor {
    /// 创建CBC模式加密器
    pub fn new(key: &[u8; 16], iv: &[u8; 16]) -> Self {
        let key = GenericArray::from_slice(key);
        let cipher = Aes128::new(key);
        
        Self {
            cipher,
            iv: *iv,
        }
    }
    
    /// CBC模式加密
    pub fn encrypt_cbc(&mut self, data: &mut [u8]) -> Result<(), &'static str> {
        if data.len() % 16 != 0 {
            return Err("Data length must be multiple of 16");
        }
        
        let mut prev_block = self.iv;
        
        for chunk in data.chunks_exact_mut(16) {
            // XOR with previous ciphertext block
            for i in 0..16 {
                chunk[i] ^= prev_block[i];
            }
            
            // Encrypt
            let mut block = [0u8; 16];
            block.copy_from_slice(chunk);
            self.encrypt_block(&mut block);
            
            // Update for next iteration
            prev_block = block;
            chunk.copy_from_slice(&block);
        }
        
        Ok(())
    }
    
    /// CBC模式解密
    pub fn decrypt_cbc(&mut self, data: &mut [u8]) -> Result<(), &'static str> {
        if data.len() % 16 != 0 {
            return Err("Data length must be multiple of 16");
        }
        
        let mut prev_block = self.iv;
        
        for chunk in data.chunks_exact_mut(16) {
            let current_cipher = *chunk;
            
            // Decrypt
            let mut block = [0u8; 16];
            block.copy_from_slice(chunk);
            self.decrypt_block(&mut block);
            
            // XOR with previous ciphertext block
            for i in 0..16 {
                block[i] ^= prev_block[i];
            }
            
            // Update for next iteration
            prev_block = current_cipher;
            chunk.copy_from_slice(&block);
        }
        
        Ok(())
    }
    
    fn encrypt_block(&self, block: &mut [u8; 16]) {
        let mut block = GenericArray::from_mut_slice(block);
        self.cipher.encrypt_block(&mut block);
    }
    
    fn decrypt_block(&self, block: &mut [u8; 16]) {
        let mut block = GenericArray::from_mut_slice(block);
        self.cipher.decrypt_block(&mut block);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_aes_ecb() {
        let key = [0u8; 16];
        let encryptor = AesEncryptor::new(&key);
        
        let mut data = [0u8; 16];
        data[0] = 0x01;
        
        let original = data;
        encryptor.encrypt_block(&mut data);
        assert_ne!(data, original);
        
        encryptor.decrypt_block(&mut data);
        assert_eq!(data, original);
    }
    
    #[test]
    fn test_aes_cbc() {
        let key = [0u8; 16];
        let iv = [1u8; 16];
        let mut encryptor = AesCbcEncryptor::new(&key, &iv);
        
        let mut data = [0u8; 32];
        data[0] = 0x01;
        data[16] = 0x02;
        
        let original = data;
        encryptor.encrypt_cbc(&mut data).unwrap();
        assert_ne!(data, original);
        
        let mut decryptor = AesCbcEncryptor::new(&key, &iv);
        decryptor.decrypt_cbc(&mut data).unwrap();
        assert_eq!(data, original);
    }
}
```

### 非对称加密

非对称加密使用公钥和私钥对，适合密钥交换和数字签名。

#### RSA加密实现

```rust
use rsa::{RsaPrivateKey, RsaPublicKey, PaddingScheme, PublicKey, PrivateKey};
use rand::rngs::OsRng;
use sha2::{Sha256, Digest};

/// RSA密钥对
pub struct RsaKeyPair {
    private_key: RsaPrivateKey,
    public_key: RsaPublicKey,
}

impl RsaKeyPair {
    /// 生成新的RSA密钥对
    pub fn generate(bits: usize) -> Result<Self, &'static str> {
        let mut rng = OsRng;
        
        let private_key = RsaPrivateKey::new(&mut rng, bits)
            .map_err(|_| "Failed to generate private key")?;
        
        let public_key = RsaPublicKey::from(&private_key);
        
        Ok(Self {
            private_key,
            public_key,
        })
    }
    
    /// 获取公钥
    pub fn public_key(&self) -> &RsaPublicKey {
        &self.public_key
    }
    
    /// 使用公钥加密
    pub fn encrypt(&self, data: &[u8]) -> Result<Vec<u8>, &'static str> {
        let mut rng = OsRng;
        let padding = PaddingScheme::new_pkcs1v15_encrypt();
        
        self.public_key
            .encrypt(&mut rng, padding, data)
            .map_err(|_| "Encryption failed")
    }
    
    /// 使用私钥解密
    pub fn decrypt(&self, ciphertext: &[u8]) -> Result<Vec<u8>, &'static str> {
        let padding = PaddingScheme::new_pkcs1v15_encrypt();
        
        self.private_key
            .decrypt(padding, ciphertext)
            .map_err(|_| "Decryption failed")
    }
    
    /// 使用私钥签名
    pub fn sign(&self, message: &[u8]) -> Result<Vec<u8>, &'static str> {
        let mut hasher = Sha256::new();
        hasher.update(message);
        let hash = hasher.finalize();
        
        let padding = PaddingScheme::new_pkcs1v15_sign(Some(rsa::Hash::SHA2_256));
        
        self.private_key
            .sign(padding, &hash)
            .map_err(|_| "Signing failed")
    }
    
    /// 使用公钥验证签名
    pub fn verify(&self, message: &[u8], signature: &[u8]) -> Result<bool, &'static str> {
        let mut hasher = Sha256::new();
        hasher.update(message);
        let hash = hasher.finalize();
        
        let padding = PaddingScheme::new_pkcs1v15_sign(Some(rsa::Hash::SHA2_256));
        
        match self.public_key.verify(padding, &hash, signature) {
            Ok(()) => Ok(true),
            Err(_) => Ok(false),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_rsa_encryption() {
        let keypair = RsaKeyPair::generate(2048).unwrap();
        let message = b"Hello, RSA!";
        
        let ciphertext = keypair.encrypt(message).unwrap();
        let plaintext = keypair.decrypt(&ciphertext).unwrap();
        
        assert_eq!(message, plaintext.as_slice());
    }
    
    #[test]
    fn test_rsa_signature() {
        let keypair = RsaKeyPair::generate(2048).unwrap();
        let message = b"Hello, RSA signature!";
        
        let signature = keypair.sign(message).unwrap();
        let is_valid = keypair.verify(message, &signature).unwrap();
        
        assert!(is_valid);
        
        // Test with wrong message
        let wrong_message = b"Wrong message";
        let is_valid = keypair.verify(wrong_message, &signature).unwrap();
        assert!(!is_valid);
    }
}
```

### 哈希函数

哈希函数用于数据完整性验证和数字签名。

#### SHA-256实现

```rust
use sha2::{Sha256, Digest};
use heapless::Vec;

/// SHA-256哈希计算器
pub struct Sha256Hasher {
    hasher: Sha256,
}

impl Sha256Hasher {
    /// 创建新的哈希计算器
    pub fn new() -> Self {
        Self {
            hasher: Sha256::new(),
        }
    }
    
    /// 添加数据到哈希计算
    pub fn update(&mut self, data: &[u8]) {
        self.hasher.update(data);
    }
    
    /// 完成哈希计算并返回结果
    pub fn finalize(self) -> [u8; 32] {
        let result = self.hasher.finalize();
        let mut hash = [0u8; 32];
        hash.copy_from_slice(&result);
        hash
    }
    
    /// 一次性计算哈希值
    pub fn hash(data: &[u8]) -> [u8; 32] {
        let mut hasher = Sha256::new();
        hasher.update(data);
        let result = hasher.finalize();
        
        let mut hash = [0u8; 32];
        hash.copy_from_slice(&result);
        hash
    }
    
    /// 计算HMAC
    pub fn hmac(key: &[u8], message: &[u8]) -> [u8; 32] {
        use hmac::{Hmac, Mac};
        type HmacSha256 = Hmac<Sha256>;
        
        let mut mac = HmacSha256::new_from_slice(key)
            .expect("HMAC can take key of any size");
        mac.update(message);
        
        let result = mac.finalize().into_bytes();
        let mut hmac = [0u8; 32];
        hmac.copy_from_slice(&result);
        hmac
    }
    
    /// 验证HMAC
    pub fn verify_hmac(key: &[u8], message: &[u8], expected: &[u8; 32]) -> bool {
        let computed = Self::hmac(key, message);
        computed == *expected
    }
}

/// 密码哈希函数 (PBKDF2)
pub struct PasswordHasher;

impl PasswordHasher {
    /// 使用PBKDF2派生密钥
    pub fn derive_key(
        password: &[u8],
        salt: &[u8],
        iterations: u32,
        output_len: usize
    ) -> Vec<u8, 64> {
        use pbkdf2::pbkdf2;
        use hmac::Hmac;
        
        let mut output = Vec::new();
        output.resize(output_len, 0).unwrap();
        
        pbkdf2::<Hmac<Sha256>>(password, salt, iterations, &mut output);
        
        output
    }
    
    /// 生成随机盐值
    pub fn generate_salt() -> [u8; 16] {
        use rand::RngCore;
        let mut salt = [0u8; 16];
        OsRng.fill_bytes(&mut salt);
        salt
    }
    
    /// 哈希密码
    pub fn hash_password(password: &[u8], salt: &[u8; 16]) -> [u8; 32] {
        let derived = Self::derive_key(password, salt, 10000, 32);
        let mut hash = [0u8; 32];
        hash.copy_from_slice(&derived);
        hash
    }
    
    /// 验证密码
    pub fn verify_password(
        password: &[u8],
        salt: &[u8; 16],
        expected_hash: &[u8; 32]
    ) -> bool {
        let computed_hash = Self::hash_password(password, salt);
        computed_hash == *expected_hash
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_sha256() {
        let data = b"Hello, SHA-256!";
        let hash1 = Sha256Hasher::hash(data);
        let hash2 = Sha256Hasher::hash(data);
        
        assert_eq!(hash1, hash2);
        
        let different_data = b"Different data";
        let hash3 = Sha256Hasher::hash(different_data);
        assert_ne!(hash1, hash3);
    }
    
    #[test]
    fn test_hmac() {
        let key = b"secret_key";
        let message = b"Hello, HMAC!";
        
        let hmac1 = Sha256Hasher::hmac(key, message);
        let hmac2 = Sha256Hasher::hmac(key, message);
        
        assert_eq!(hmac1, hmac2);
        assert!(Sha256Hasher::verify_hmac(key, message, &hmac1));
        
        let wrong_key = b"wrong_key";
        assert!(!Sha256Hasher::verify_hmac(wrong_key, message, &hmac1));
    }
    
    #[test]
    fn test_password_hashing() {
        let password = b"my_secure_password";
        let salt = PasswordHasher::generate_salt();
        
        let hash1 = PasswordHasher::hash_password(password, &salt);
        let hash2 = PasswordHasher::hash_password(password, &salt);
        
        assert_eq!(hash1, hash2);
        assert!(PasswordHasher::verify_password(password, &salt, &hash1));
        
        let wrong_password = b"wrong_password";
        assert!(!PasswordHasher::verify_password(wrong_password, &salt, &hash1));
    }
}
```

## 随机数生成

安全的随机数生成对密码学应用至关重要。

### 硬件随机数生成器

```rust
use stm32f4xx_hal::{pac, rng::Rng};

/// 硬件随机数生成器
pub struct HardwareRng {
    rng: Rng,
}

impl HardwareRng {
    /// 初始化硬件RNG
    pub fn new(rng_peripheral: pac::RNG, rcc: &mut pac::RCC) -> Self {
        // 启用RNG时钟
        rcc.ahb2enr.modify(|_, w| w.rngen().set_bit());
        
        let rng = Rng::new(rng_peripheral);
        
        Self { rng }
    }
    
    /// 生成32位随机数
    pub fn gen_u32(&mut self) -> Result<u32, &'static str> {
        self.rng.gen().map_err(|_| "RNG error")
    }
    
    /// 填充随机字节
    pub fn fill_bytes(&mut self, dest: &mut [u8]) -> Result<(), &'static str> {
        for chunk in dest.chunks_mut(4) {
            let random = self.gen_u32()?;
            let bytes = random.to_le_bytes();
            
            for (i, &byte) in bytes.iter().enumerate() {
                if i < chunk.len() {
                    chunk[i] = byte;
                }
            }
        }
        
        Ok(())
    }
    
    /// 生成随机密钥
    pub fn generate_key<const N: usize>(&mut self) -> Result<[u8; N], &'static str> {
        let mut key = [0u8; N];
        self.fill_bytes(&mut key)?;
        Ok(key)
    }
}

/// 伪随机数生成器 (用于测试)
pub struct PseudoRng {
    state: u64,
}

impl PseudoRng {
    /// 创建新的伪随机数生成器
    pub fn new(seed: u64) -> Self {
        Self { state: seed }
    }
    
    /// 生成下一个随机数 (Linear Congruential Generator)
    pub fn next(&mut self) -> u32 {
        self.state = self.state.wrapping_mul(1103515245).wrapping_add(12345);
        (self.state >> 16) as u32
    }
    
    /// 填充随机字节
    pub fn fill_bytes(&mut self, dest: &mut [u8]) {
        for chunk in dest.chunks_mut(4) {
            let random = self.next();
            let bytes = random.to_le_bytes();
            
            for (i, &byte) in bytes.iter().enumerate() {
                if i < chunk.len() {
                    chunk[i] = byte;
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_pseudo_rng() {
        let mut rng = PseudoRng::new(12345);
        
        let num1 = rng.next();
        let num2 = rng.next();
        
        assert_ne!(num1, num2);
        
        // Test reproducibility
        let mut rng2 = PseudoRng::new(12345);
        assert_eq!(num1, rng2.next());
        assert_eq!(num2, rng2.next());
    }
    
    #[test]
    fn test_fill_bytes() {
        let mut rng = PseudoRng::new(54321);
        let mut buffer = [0u8; 10];
        
        rng.fill_bytes(&mut buffer);
        
        // Check that buffer is not all zeros
        assert!(buffer.iter().any(|&x| x != 0));
    }
}
```

## 总结

本章介绍了嵌入式系统中的基础密码学概念和实现：

1. **对称加密** - AES算法的ECB和CBC模式实现
2. **非对称加密** - RSA加密和数字签名
3. **哈希函数** - SHA-256和HMAC实现
4. **密码哈希** - PBKDF2密钥派生
5. **随机数生成** - 硬件RNG和伪随机数生成器

这些基础组件为构建安全的嵌入式系统提供了必要的工具。在下一章中，我们将学习如何在实际应用中使用这些密码学原语。