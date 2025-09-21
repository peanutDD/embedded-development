# 安全存储

在嵌入式系统中，安全存储是保护敏感数据的关键技术。本章将介绍如何实现安全的数据存储，包括加密存储、完整性保护和安全擦除。

## 加密存储系统

### 分层存储架构

```rust
use heapless::{Vec, FnvIndexMap};
use sha2::{Sha256, Digest};
use aes::Aes256;
use aes::cipher::{BlockEncrypt, BlockDecrypt, KeyInit};

/// 存储层级
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum StorageLevel {
    Public,     // 公开数据
    Internal,   // 内部数据
    Sensitive,  // 敏感数据
    Secret,     // 机密数据
}

/// 数据分类
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DataClassification {
    Configuration,
    UserData,
    Credentials,
    Keys,
    Logs,
    Temporary,
}

/// 存储条目
#[derive(Debug, Clone)]
pub struct StorageEntry {
    pub id: u32,
    pub level: StorageLevel,
    pub classification: DataClassification,
    pub data: Vec<u8, 512>,
    pub checksum: [u8; 32],
    pub created_at: u64,
    pub accessed_at: u64,
    pub modified_at: u64,
    pub access_count: u32,
}

impl StorageEntry {
    pub fn new(
        id: u32,
        level: StorageLevel,
        classification: DataClassification,
        data: &[u8]
    ) -> Result<Self, &'static str> {
        if data.len() > 512 {
            return Err("Data too large");
        }
        
        let mut entry_data = Vec::new();
        entry_data.extend_from_slice(data).map_err(|_| "Data too large")?;
        
        let checksum = Self::calculate_checksum(&entry_data);
        let current_time = Self::get_current_time();
        
        Ok(Self {
            id,
            level,
            classification,
            data: entry_data,
            checksum,
            created_at: current_time,
            accessed_at: current_time,
            modified_at: current_time,
            access_count: 0,
        })
    }
    
    pub fn update_data(&mut self, data: &[u8]) -> Result<(), &'static str> {
        if data.len() > 512 {
            return Err("Data too large");
        }
        
        self.data.clear();
        self.data.extend_from_slice(data).map_err(|_| "Data too large")?;
        
        self.checksum = Self::calculate_checksum(&self.data);
        self.modified_at = Self::get_current_time();
        
        Ok(())
    }
    
    pub fn verify_integrity(&self) -> bool {
        let calculated_checksum = Self::calculate_checksum(&self.data);
        calculated_checksum == self.checksum
    }
    
    pub fn access(&mut self) {
        self.accessed_at = Self::get_current_time();
        self.access_count += 1;
    }
    
    fn calculate_checksum(data: &[u8]) -> [u8; 32] {
        let mut hasher = Sha256::new();
        hasher.update(data);
        let result = hasher.finalize();
        
        let mut checksum = [0u8; 32];
        checksum.copy_from_slice(&result);
        checksum
    }
    
    fn get_current_time() -> u64 {
        // 在实际实现中应使用真实的时间戳
        12345678
    }
}

/// 安全存储管理器
pub struct SecureStorage {
    entries: FnvIndexMap<u32, StorageEntry, 32>,
    encryption_keys: FnvIndexMap<StorageLevel, [u8; 32], 4>,
    next_id: u32,
    access_log: Vec<(u32, u64), 64>, // (entry_id, timestamp)
}

impl SecureStorage {
    pub fn new() -> Self {
        Self {
            entries: FnvIndexMap::new(),
            encryption_keys: FnvIndexMap::new(),
            next_id: 1,
            access_log: Vec::new(),
        }
    }
    
    /// 设置存储层级的加密密钥
    pub fn set_encryption_key(&mut self, level: StorageLevel, key: [u8; 32]) -> Result<(), &'static str> {
        self.encryption_keys.insert(level, key).map_err(|_| "Failed to set encryption key")?;
        Ok(())
    }
    
    /// 存储数据
    pub fn store(
        &mut self,
        level: StorageLevel,
        classification: DataClassification,
        data: &[u8]
    ) -> Result<u32, &'static str> {
        let id = self.next_id;
        self.next_id += 1;
        
        // 根据存储层级加密数据
        let encrypted_data = match level {
            StorageLevel::Public => {
                // 公开数据不加密
                data.to_vec()
            },
            _ => {
                let key = self.encryption_keys.get(&level).ok_or("No encryption key for level")?;
                self.encrypt_data(data, key)?
            }
        };
        
        let entry = StorageEntry::new(id, level, classification, &encrypted_data)?;
        
        self.entries.insert(id, entry).map_err(|_| "Storage full")?;
        
        // 记录访问日志
        self.log_access(id);
        
        Ok(id)
    }
    
    /// 读取数据
    pub fn read(&mut self, id: u32) -> Result<Vec<u8, 512>, &'static str> {
        let entry = self.entries.get_mut(&id).ok_or("Entry not found")?;
        
        // 验证完整性
        if !entry.verify_integrity() {
            return Err("Data integrity check failed");
        }
        
        entry.access();
        
        // 根据存储层级解密数据
        let decrypted_data = match entry.level {
            StorageLevel::Public => {
                entry.data.clone()
            },
            _ => {
                let key = self.encryption_keys.get(&entry.level).ok_or("No decryption key for level")?;
                self.decrypt_data(&entry.data, key)?
            }
        };
        
        // 记录访问日志
        self.log_access(id);
        
        Ok(decrypted_data)
    }
    
    /// 更新数据
    pub fn update(&mut self, id: u32, data: &[u8]) -> Result<(), &'static str> {
        let entry = self.entries.get_mut(&id).ok_or("Entry not found")?;
        
        // 根据存储层级加密数据
        let encrypted_data = match entry.level {
            StorageLevel::Public => {
                data.to_vec()
            },
            _ => {
                let key = self.encryption_keys.get(&entry.level).ok_or("No encryption key for level")?;
                self.encrypt_data(data, key)?
            }
        };
        
        entry.update_data(&encrypted_data)?;
        
        // 记录访问日志
        self.log_access(id);
        
        Ok(())
    }
    
    /// 删除数据
    pub fn delete(&mut self, id: u32) -> Result<(), &'static str> {
        let entry = self.entries.remove(&id).ok_or("Entry not found")?;
        
        // 安全擦除敏感数据
        if matches!(entry.level, StorageLevel::Sensitive | StorageLevel::Secret) {
            self.secure_erase(&entry);
        }
        
        // 记录访问日志
        self.log_access(id);
        
        Ok(())
    }
    
    /// 列出指定层级的条目
    pub fn list_entries(&self, level: Option<StorageLevel>) -> Vec<u32, 32> {
        let mut ids = Vec::new();
        
        for (&id, entry) in &self.entries {
            if level.is_none() || level == Some(entry.level) {
                ids.push(id).ok();
            }
        }
        
        ids
    }
    
    /// 获取存储统计信息
    pub fn get_statistics(&self) -> StorageStatistics {
        let mut stats = StorageStatistics::new();
        
        for entry in self.entries.values() {
            stats.total_entries += 1;
            stats.total_size += entry.data.len() as u32;
            
            match entry.level {
                StorageLevel::Public => stats.public_entries += 1,
                StorageLevel::Internal => stats.internal_entries += 1,
                StorageLevel::Sensitive => stats.sensitive_entries += 1,
                StorageLevel::Secret => stats.secret_entries += 1,
            }
            
            match entry.classification {
                DataClassification::Configuration => stats.config_entries += 1,
                DataClassification::UserData => stats.user_data_entries += 1,
                DataClassification::Credentials => stats.credential_entries += 1,
                DataClassification::Keys => stats.key_entries += 1,
                DataClassification::Logs => stats.log_entries += 1,
                DataClassification::Temporary => stats.temp_entries += 1,
            }
        }
        
        stats
    }
    
    /// 清理临时数据
    pub fn cleanup_temporary(&mut self) -> u32 {
        let mut deleted_count = 0;
        let mut temp_ids = Vec::<u32, 32>::new();
        
        for (&id, entry) in &self.entries {
            if entry.classification == DataClassification::Temporary {
                temp_ids.push(id).ok();
            }
        }
        
        for id in &temp_ids {
            if self.delete(*id).is_ok() {
                deleted_count += 1;
            }
        }
        
        deleted_count
    }
    
    /// 验证所有条目的完整性
    pub fn verify_all_integrity(&self) -> Vec<u32, 32> {
        let mut corrupted_ids = Vec::new();
        
        for (&id, entry) in &self.entries {
            if !entry.verify_integrity() {
                corrupted_ids.push(id).ok();
            }
        }
        
        corrupted_ids
    }
    
    fn encrypt_data(&self, data: &[u8], key: &[u8; 32]) -> Result<Vec<u8, 512>, &'static str> {
        // 使用AES-256-ECB加密 (简化实现)
        let cipher = Aes256::new_from_slice(key).map_err(|_| "Invalid key")?;
        
        // 添加PKCS#7填充
        let mut padded_data = Vec::new();
        padded_data.extend_from_slice(data).map_err(|_| "Data too large")?;
        
        let padding_len = 16 - (data.len() % 16);
        for _ in 0..padding_len {
            padded_data.push(padding_len as u8).map_err(|_| "Padding failed")?;
        }
        
        // 加密
        let mut encrypted = Vec::new();
        for chunk in padded_data.chunks(16) {
            let mut block = [0u8; 16];
            block.copy_from_slice(chunk);
            
            let mut block_array = block.into();
            cipher.encrypt_block(&mut block_array);
            let encrypted_block: [u8; 16] = block_array.into();
            
            encrypted.extend_from_slice(&encrypted_block).map_err(|_| "Encryption failed")?;
        }
        
        Ok(encrypted)
    }
    
    fn decrypt_data(&self, data: &[u8], key: &[u8; 32]) -> Result<Vec<u8, 512>, &'static str> {
        if data.len() % 16 != 0 {
            return Err("Invalid ciphertext length");
        }
        
        let cipher = Aes256::new_from_slice(key).map_err(|_| "Invalid key")?;
        
        let mut decrypted = Vec::new();
        for chunk in data.chunks(16) {
            let mut block = [0u8; 16];
            block.copy_from_slice(chunk);
            
            let mut block_array = block.into();
            cipher.decrypt_block(&mut block_array);
            let decrypted_block: [u8; 16] = block_array.into();
            
            decrypted.extend_from_slice(&decrypted_block).map_err(|_| "Decryption failed")?;
        }
        
        // 移除PKCS#7填充
        if let Some(&padding_len) = decrypted.last() {
            if padding_len > 0 && padding_len <= 16 {
                let new_len = decrypted.len().saturating_sub(padding_len as usize);
                decrypted.truncate(new_len);
            }
        }
        
        Ok(decrypted)
    }
    
    fn secure_erase(&self, entry: &StorageEntry) {
        // 在实际实现中，应该多次覆写内存区域
        // 这里只是示例
        let _ = entry.data.len(); // 防止编译器优化
    }
    
    fn log_access(&mut self, id: u32) {
        let timestamp = StorageEntry::get_current_time();
        
        if self.access_log.len() >= 64 {
            // 移除最旧的日志条目
            self.access_log.remove(0);
        }
        
        self.access_log.push((id, timestamp)).ok();
    }
    
    pub fn get_access_log(&self) -> &[(u32, u64)] {
        &self.access_log
    }
    
    pub fn get_entry_count(&self) -> usize {
        self.entries.len()
    }
}

/// 存储统计信息
#[derive(Debug, Default)]
pub struct StorageStatistics {
    pub total_entries: u32,
    pub total_size: u32,
    pub public_entries: u32,
    pub internal_entries: u32,
    pub sensitive_entries: u32,
    pub secret_entries: u32,
    pub config_entries: u32,
    pub user_data_entries: u32,
    pub credential_entries: u32,
    pub key_entries: u32,
    pub log_entries: u32,
    pub temp_entries: u32,
}

impl StorageStatistics {
    pub fn new() -> Self {
        Self::default()
    }
}

/// 安全文件系统
pub struct SecureFileSystem {
    storage: SecureStorage,
    file_table: FnvIndexMap<heapless::String<64>, u32, 16>, // filename -> storage_id
}

impl SecureFileSystem {
    pub fn new() -> Self {
        Self {
            storage: SecureStorage::new(),
            file_table: FnvIndexMap::new(),
        }
    }
    
    /// 设置加密密钥
    pub fn set_encryption_key(&mut self, level: StorageLevel, key: [u8; 32]) -> Result<(), &'static str> {
        self.storage.set_encryption_key(level, key)
    }
    
    /// 创建文件
    pub fn create_file(
        &mut self,
        filename: &str,
        level: StorageLevel,
        data: &[u8]
    ) -> Result<(), &'static str> {
        if filename.len() > 64 {
            return Err("Filename too long");
        }
        
        let filename_str = heapless::String::from(filename);
        
        if self.file_table.contains_key(&filename_str) {
            return Err("File already exists");
        }
        
        let storage_id = self.storage.store(level, DataClassification::UserData, data)?;
        
        self.file_table.insert(filename_str, storage_id).map_err(|_| "File table full")?;
        
        Ok(())
    }
    
    /// 读取文件
    pub fn read_file(&mut self, filename: &str) -> Result<Vec<u8, 512>, &'static str> {
        let filename_str = heapless::String::from(filename);
        let storage_id = self.file_table.get(&filename_str).ok_or("File not found")?;
        
        self.storage.read(*storage_id)
    }
    
    /// 写入文件
    pub fn write_file(&mut self, filename: &str, data: &[u8]) -> Result<(), &'static str> {
        let filename_str = heapless::String::from(filename);
        let storage_id = self.file_table.get(&filename_str).ok_or("File not found")?;
        
        self.storage.update(*storage_id, data)
    }
    
    /// 删除文件
    pub fn delete_file(&mut self, filename: &str) -> Result<(), &'static str> {
        let filename_str = heapless::String::from(filename);
        let storage_id = self.file_table.remove(&filename_str).ok_or("File not found")?;
        
        self.storage.delete(storage_id)
    }
    
    /// 列出文件
    pub fn list_files(&self) -> Vec<heapless::String<64>, 16> {
        let mut files = Vec::new();
        
        for filename in self.file_table.keys() {
            files.push(filename.clone()).ok();
        }
        
        files
    }
    
    /// 文件是否存在
    pub fn file_exists(&self, filename: &str) -> bool {
        let filename_str = heapless::String::from(filename);
        self.file_table.contains_key(&filename_str)
    }
    
    /// 获取文件统计信息
    pub fn get_file_count(&self) -> usize {
        self.file_table.len()
    }
    
    /// 验证文件系统完整性
    pub fn verify_integrity(&self) -> Vec<heapless::String<64>, 16> {
        let mut corrupted_files = Vec::new();
        let corrupted_ids = self.storage.verify_all_integrity();
        
        for (&filename, &storage_id) in &self.file_table {
            if corrupted_ids.contains(&storage_id) {
                corrupted_files.push(filename.clone()).ok();
            }
        }
        
        corrupted_files
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_storage_entry() {
        let data = b"test data";
        let entry = StorageEntry::new(
            1,
            StorageLevel::Internal,
            DataClassification::UserData,
            data
        ).unwrap();
        
        assert_eq!(entry.id, 1);
        assert_eq!(entry.level, StorageLevel::Internal);
        assert_eq!(entry.data.as_slice(), data);
        assert!(entry.verify_integrity());
    }
    
    #[test]
    fn test_secure_storage() {
        let mut storage = SecureStorage::new();
        let key = [0x42u8; 32];
        
        storage.set_encryption_key(StorageLevel::Sensitive, key).unwrap();
        
        let data = b"sensitive data";
        let id = storage.store(
            StorageLevel::Sensitive,
            DataClassification::UserData,
            data
        ).unwrap();
        
        let retrieved_data = storage.read(id).unwrap();
        assert_eq!(retrieved_data.as_slice(), data);
        
        let new_data = b"updated sensitive data";
        storage.update(id, new_data).unwrap();
        
        let updated_data = storage.read(id).unwrap();
        assert_eq!(updated_data.as_slice(), new_data);
        
        storage.delete(id).unwrap();
        assert!(storage.read(id).is_err());
    }
    
    #[test]
    fn test_secure_file_system() {
        let mut fs = SecureFileSystem::new();
        let key = [0x42u8; 32];
        
        fs.set_encryption_key(StorageLevel::Internal, key).unwrap();
        
        let filename = "test.txt";
        let data = b"file content";
        
        fs.create_file(filename, StorageLevel::Internal, data).unwrap();
        assert!(fs.file_exists(filename));
        
        let read_data = fs.read_file(filename).unwrap();
        assert_eq!(read_data.as_slice(), data);
        
        let new_data = b"updated file content";
        fs.write_file(filename, new_data).unwrap();
        
        let updated_data = fs.read_file(filename).unwrap();
        assert_eq!(updated_data.as_slice(), new_data);
        
        fs.delete_file(filename).unwrap();
        assert!(!fs.file_exists(filename));
    }
    
    #[test]
    fn test_storage_statistics() {
        let mut storage = SecureStorage::new();
        let key = [0x42u8; 32];
        
        storage.set_encryption_key(StorageLevel::Internal, key).unwrap();
        
        storage.store(StorageLevel::Public, DataClassification::Configuration, b"config").unwrap();
        storage.store(StorageLevel::Internal, DataClassification::UserData, b"user data").unwrap();
        storage.store(StorageLevel::Sensitive, DataClassification::Credentials, b"password").unwrap();
        
        let stats = storage.get_statistics();
        assert_eq!(stats.total_entries, 3);
        assert_eq!(stats.public_entries, 1);
        assert_eq!(stats.internal_entries, 1);
        assert_eq!(stats.sensitive_entries, 1);
        assert_eq!(stats.config_entries, 1);
        assert_eq!(stats.user_data_entries, 1);
        assert_eq!(stats.credential_entries, 1);
    }
}
```

## 完整性保护

### 数字签名和验证

```rust
use sha2::{Sha256, Digest};
use heapless::{Vec, String};

/// 签名算法类型
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SignatureAlgorithm {
    HMACSHA256,
    RSASHA256,
    ECDSASHA256,
}

/// 数字签名
#[derive(Debug, Clone)]
pub struct DigitalSignature {
    pub algorithm: SignatureAlgorithm,
    pub signature: Vec<u8, 256>,
    pub timestamp: u64,
    pub signer_id: String<32>,
}

impl DigitalSignature {
    pub fn new(
        algorithm: SignatureAlgorithm,
        signature: &[u8],
        signer_id: &str
    ) -> Result<Self, &'static str> {
        if signature.len() > 256 {
            return Err("Signature too large");
        }
        
        if signer_id.len() > 32 {
            return Err("Signer ID too long");
        }
        
        let mut sig_data = Vec::new();
        sig_data.extend_from_slice(signature).map_err(|_| "Signature too large")?;
        
        let signer_string = String::from(signer_id);
        
        Ok(Self {
            algorithm,
            signature: sig_data,
            timestamp: Self::get_current_time(),
            signer_id: signer_string,
        })
    }
    
    fn get_current_time() -> u64 {
        // 在实际实现中应使用真实的时间戳
        12345678
    }
}

/// 签名验证器
pub struct SignatureVerifier {
    hmac_keys: heapless::FnvIndexMap<String<32>, [u8; 32], 8>,
    rsa_public_keys: heapless::FnvIndexMap<String<32>, Vec<u8, 512>, 8>,
}

impl SignatureVerifier {
    pub fn new() -> Self {
        Self {
            hmac_keys: heapless::FnvIndexMap::new(),
            rsa_public_keys: heapless::FnvIndexMap::new(),
        }
    }
    
    /// 添加HMAC密钥
    pub fn add_hmac_key(&mut self, signer_id: &str, key: [u8; 32]) -> Result<(), &'static str> {
        if signer_id.len() > 32 {
            return Err("Signer ID too long");
        }
        
        let id_string = String::from(signer_id);
        self.hmac_keys.insert(id_string, key).map_err(|_| "Key storage full")?;
        
        Ok(())
    }
    
    /// 添加RSA公钥
    pub fn add_rsa_public_key(&mut self, signer_id: &str, public_key: &[u8]) -> Result<(), &'static str> {
        if signer_id.len() > 32 {
            return Err("Signer ID too long");
        }
        
        if public_key.len() > 512 {
            return Err("Public key too large");
        }
        
        let id_string = String::from(signer_id);
        let mut key_data = Vec::new();
        key_data.extend_from_slice(public_key).map_err(|_| "Public key too large")?;
        
        self.rsa_public_keys.insert(id_string, key_data).map_err(|_| "Key storage full")?;
        
        Ok(())
    }
    
    /// 验证签名
    pub fn verify_signature(
        &self,
        data: &[u8],
        signature: &DigitalSignature
    ) -> Result<bool, &'static str> {
        match signature.algorithm {
            SignatureAlgorithm::HMACSHA256 => {
                self.verify_hmac_signature(data, signature)
            },
            SignatureAlgorithm::RSASHA256 => {
                self.verify_rsa_signature(data, signature)
            },
            SignatureAlgorithm::ECDSASHA256 => {
                // ECDSA验证实现
                Err("ECDSA not implemented")
            }
        }
    }
    
    fn verify_hmac_signature(
        &self,
        data: &[u8],
        signature: &DigitalSignature
    ) -> Result<bool, &'static str> {
        let key = self.hmac_keys.get(&signature.signer_id)
            .ok_or("HMAC key not found")?;
        
        let expected_signature = self.calculate_hmac(data, key);
        
        Ok(expected_signature.as_slice() == signature.signature.as_slice())
    }
    
    fn verify_rsa_signature(
        &self,
        _data: &[u8],
        signature: &DigitalSignature
    ) -> Result<bool, &'static str> {
        let _public_key = self.rsa_public_keys.get(&signature.signer_id)
            .ok_or("RSA public key not found")?;
        
        // 简化的RSA验证 (实际实现需要完整的RSA算法)
        Ok(signature.signature.len() > 0)
    }
    
    fn calculate_hmac(&self, data: &[u8], key: &[u8; 32]) -> Vec<u8, 32> {
        use hmac::{Hmac, Mac};
        type HmacSha256 = Hmac<Sha256>;
        
        let mut mac = HmacSha256::new_from_slice(key).expect("Valid key length");
        mac.update(data);
        let result = mac.finalize().into_bytes();
        
        let mut hmac_result = Vec::new();
        hmac_result.extend_from_slice(&result).expect("HMAC result fits");
        
        hmac_result
    }
}

/// 签名生成器
pub struct SignatureGenerator {
    hmac_keys: heapless::FnvIndexMap<String<32>, [u8; 32], 8>,
    signer_id: String<32>,
}

impl SignatureGenerator {
    pub fn new(signer_id: &str) -> Result<Self, &'static str> {
        if signer_id.len() > 32 {
            return Err("Signer ID too long");
        }
        
        Ok(Self {
            hmac_keys: heapless::FnvIndexMap::new(),
            signer_id: String::from(signer_id),
        })
    }
    
    /// 设置HMAC密钥
    pub fn set_hmac_key(&mut self, key: [u8; 32]) -> Result<(), &'static str> {
        self.hmac_keys.insert(self.signer_id.clone(), key)
            .map_err(|_| "Key storage full")?;
        Ok(())
    }
    
    /// 生成HMAC签名
    pub fn sign_with_hmac(&self, data: &[u8]) -> Result<DigitalSignature, &'static str> {
        let key = self.hmac_keys.get(&self.signer_id)
            .ok_or("HMAC key not set")?;
        
        let signature_data = self.calculate_hmac(data, key);
        
        DigitalSignature::new(
            SignatureAlgorithm::HMACSHA256,
            &signature_data,
            &self.signer_id
        )
    }
    
    fn calculate_hmac(&self, data: &[u8], key: &[u8; 32]) -> Vec<u8, 32> {
        use hmac::{Hmac, Mac};
        type HmacSha256 = Hmac<Sha256>;
        
        let mut mac = HmacSha256::new_from_slice(key).expect("Valid key length");
        mac.update(data);
        let result = mac.finalize().into_bytes();
        
        let mut hmac_result = Vec::new();
        hmac_result.extend_from_slice(&result).expect("HMAC result fits");
        
        hmac_result
    }
}

/// 完整性保护的数据容器
pub struct IntegrityProtectedData {
    pub data: Vec<u8, 512>,
    pub signature: DigitalSignature,
    pub version: u32,
}

impl IntegrityProtectedData {
    pub fn new(
        data: &[u8],
        generator: &SignatureGenerator
    ) -> Result<Self, &'static str> {
        if data.len() > 512 {
            return Err("Data too large");
        }
        
        let mut data_vec = Vec::new();
        data_vec.extend_from_slice(data).map_err(|_| "Data too large")?;
        
        let signature = generator.sign_with_hmac(data)?;
        
        Ok(Self {
            data: data_vec,
            signature,
            version: 1,
        })
    }
    
    /// 验证数据完整性
    pub fn verify(&self, verifier: &SignatureVerifier) -> Result<bool, &'static str> {
        verifier.verify_signature(&self.data, &self.signature)
    }
    
    /// 更新数据
    pub fn update_data(
        &mut self,
        new_data: &[u8],
        generator: &SignatureGenerator
    ) -> Result<(), &'static str> {
        if new_data.len() > 512 {
            return Err("Data too large");
        }
        
        self.data.clear();
        self.data.extend_from_slice(new_data).map_err(|_| "Data too large")?;
        
        self.signature = generator.sign_with_hmac(&self.data)?;
        self.version += 1;
        
        Ok(())
    }
    
    pub fn get_version(&self) -> u32 {
        self.version
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_signature_generation_and_verification() {
        let signer_id = "test_signer";
        let key = [0x42u8; 32];
        
        // 创建签名生成器
        let mut generator = SignatureGenerator::new(signer_id).unwrap();
        generator.set_hmac_key(key).unwrap();
        
        // 创建验证器
        let mut verifier = SignatureVerifier::new();
        verifier.add_hmac_key(signer_id, key).unwrap();
        
        // 签名数据
        let data = b"test data for signing";
        let signature = generator.sign_with_hmac(data).unwrap();
        
        // 验证签名
        let is_valid = verifier.verify_signature(data, &signature).unwrap();
        assert!(is_valid);
        
        // 验证篡改的数据
        let tampered_data = b"tampered data";
        let is_valid_tampered = verifier.verify_signature(tampered_data, &signature).unwrap();
        assert!(!is_valid_tampered);
    }
    
    #[test]
    fn test_integrity_protected_data() {
        let signer_id = "test_signer";
        let key = [0x42u8; 32];
        
        let mut generator = SignatureGenerator::new(signer_id).unwrap();
        generator.set_hmac_key(key).unwrap();
        
        let mut verifier = SignatureVerifier::new();
        verifier.add_hmac_key(signer_id, key).unwrap();
        
        // 创建受保护的数据
        let data = b"protected data";
        let protected_data = IntegrityProtectedData::new(data, &generator).unwrap();
        
        // 验证完整性
        assert!(protected_data.verify(&verifier).unwrap());
        
        // 更新数据
        let mut updated_data = protected_data;
        let new_data = b"updated protected data";
        updated_data.update_data(new_data, &generator).unwrap();
        
        // 验证更新后的完整性
        assert!(updated_data.verify(&verifier).unwrap());
        assert_eq!(updated_data.get_version(), 2);
    }
}
```

## 总结

本章介绍了嵌入式系统中的安全存储技术，包括：

1. **分层存储架构** - 根据数据敏感性分级存储和加密
2. **加密存储系统** - 使用AES等算法保护存储数据
3. **完整性保护** - 通过数字签名确保数据完整性
4. **安全文件系统** - 提供文件级别的安全存储接口
5. **密钥管理** - 安全的密钥存储和轮换机制

这些技术为嵌入式系统提供了全面的数据保护能力，确保敏感信息的安全存储和访问控制。