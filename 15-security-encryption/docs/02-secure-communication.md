# 安全通信协议

在嵌入式系统中，安全通信是保护数据传输的关键。本章将介绍如何实现安全的通信协议，包括TLS/SSL、安全认证和密钥管理。

## TLS/SSL协议

### TLS握手过程

TLS (Transport Layer Security) 是一种广泛使用的安全通信协议。

```rust
use heapless::{String, Vec};
use sha2::{Sha256, Digest};
use aes::Aes128;
use rsa::{RsaPrivateKey, RsaPublicKey};

/// TLS版本
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TlsVersion {
    Tls12,
    Tls13,
}

/// TLS消息类型
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TlsMessageType {
    ClientHello = 1,
    ServerHello = 2,
    Certificate = 11,
    ServerKeyExchange = 12,
    CertificateRequest = 13,
    ServerHelloDone = 14,
    CertificateVerify = 15,
    ClientKeyExchange = 16,
    Finished = 20,
    ApplicationData = 23,
}

/// TLS握手状态
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TlsHandshakeState {
    Start,
    ClientHelloSent,
    ServerHelloReceived,
    CertificateReceived,
    KeyExchangeReceived,
    ServerHelloDoneReceived,
    ClientKeyExchangeSent,
    ChangeCipherSpecSent,
    FinishedSent,
    HandshakeComplete,
}

/// TLS会话信息
#[derive(Debug, Clone)]
pub struct TlsSession {
    pub session_id: Vec<u8, 32>,
    pub master_secret: [u8; 48],
    pub client_random: [u8; 32],
    pub server_random: [u8; 32],
    pub cipher_suite: u16,
    pub compression_method: u8,
}

impl TlsSession {
    pub fn new() -> Self {
        Self {
            session_id: Vec::new(),
            master_secret: [0u8; 48],
            client_random: [0u8; 32],
            server_random: [0u8; 32],
            cipher_suite: 0,
            compression_method: 0,
        }
    }
    
    /// 生成主密钥
    pub fn generate_master_secret(&mut self, pre_master_secret: &[u8; 48]) {
        // PRF (Pseudo-Random Function) for TLS 1.2
        let seed = [
            b"master secret",
            &self.client_random,
            &self.server_random
        ].concat();
        
        self.master_secret = self.prf(pre_master_secret, &seed, 48);
    }
    
    /// 派生密钥材料
    pub fn derive_keys(&self) -> TlsKeys {
        let seed = [
            b"key expansion",
            &self.server_random,
            &self.client_random
        ].concat();
        
        let key_material = self.prf(&self.master_secret, &seed, 104);
        
        TlsKeys::from_key_material(&key_material)
    }
    
    /// TLS PRF函数
    fn prf(&self, secret: &[u8], seed: &[u8], length: usize) -> [u8; 48] {
        // 简化的PRF实现 (实际应使用HMAC-SHA256)
        let mut result = [0u8; 48];
        let mut hasher = Sha256::new();
        hasher.update(secret);
        hasher.update(seed);
        let hash = hasher.finalize();
        
        for i in 0..length.min(48) {
            result[i] = hash[i % 32];
        }
        
        result
    }
}

/// TLS密钥材料
#[derive(Debug)]
pub struct TlsKeys {
    pub client_write_mac_key: [u8; 20],
    pub server_write_mac_key: [u8; 20],
    pub client_write_key: [u8; 16],
    pub server_write_key: [u8; 16],
    pub client_write_iv: [u8; 16],
    pub server_write_iv: [u8; 16],
}

impl TlsKeys {
    pub fn from_key_material(material: &[u8]) -> Self {
        let mut keys = Self {
            client_write_mac_key: [0u8; 20],
            server_write_mac_key: [0u8; 20],
            client_write_key: [0u8; 16],
            server_write_key: [0u8; 16],
            client_write_iv: [0u8; 16],
            server_write_iv: [0u8; 16],
        };
        
        let mut offset = 0;
        
        // 提取各种密钥
        keys.client_write_mac_key.copy_from_slice(&material[offset..offset+20]);
        offset += 20;
        
        keys.server_write_mac_key.copy_from_slice(&material[offset..offset+20]);
        offset += 20;
        
        keys.client_write_key.copy_from_slice(&material[offset..offset+16]);
        offset += 16;
        
        keys.server_write_key.copy_from_slice(&material[offset..offset+16]);
        offset += 16;
        
        keys.client_write_iv.copy_from_slice(&material[offset..offset+16]);
        offset += 16;
        
        keys.server_write_iv.copy_from_slice(&material[offset..offset+16]);
        
        keys
    }
}

/// TLS客户端
pub struct TlsClient {
    state: TlsHandshakeState,
    session: TlsSession,
    keys: Option<TlsKeys>,
    sequence_number: u64,
}

impl TlsClient {
    pub fn new() -> Self {
        Self {
            state: TlsHandshakeState::Start,
            session: TlsSession::new(),
            keys: None,
            sequence_number: 0,
        }
    }
    
    /// 开始TLS握手
    pub fn start_handshake(&mut self) -> Result<Vec<u8, 256>, &'static str> {
        if self.state != TlsHandshakeState::Start {
            return Err("Handshake already started");
        }
        
        // 生成客户端随机数
        self.generate_client_random();
        
        // 构建ClientHello消息
        let client_hello = self.build_client_hello()?;
        
        self.state = TlsHandshakeState::ClientHelloSent;
        
        Ok(client_hello)
    }
    
    /// 处理服务器响应
    pub fn process_server_message(&mut self, message: &[u8]) -> Result<Option<Vec<u8, 256>>, &'static str> {
        if message.len() < 5 {
            return Err("Message too short");
        }
        
        let msg_type = message[0];
        let length = u32::from_be_bytes([0, message[2], message[3], message[4]]) as usize;
        
        if message.len() < 5 + length {
            return Err("Incomplete message");
        }
        
        let payload = &message[5..5+length];
        
        match msg_type {
            2 => self.process_server_hello(payload),
            11 => self.process_certificate(payload),
            12 => self.process_server_key_exchange(payload),
            14 => self.process_server_hello_done(payload),
            20 => self.process_finished(payload),
            _ => Err("Unknown message type"),
        }
    }
    
    /// 加密应用数据
    pub fn encrypt_data(&mut self, data: &[u8]) -> Result<Vec<u8, 512>, &'static str> {
        if self.state != TlsHandshakeState::HandshakeComplete {
            return Err("Handshake not complete");
        }
        
        let keys = self.keys.as_ref().ok_or("Keys not available")?;
        
        // 构建TLS记录
        let mut record = Vec::new();
        
        // TLS记录头
        record.push(TlsMessageType::ApplicationData as u8).ok();
        record.push(0x03).ok(); // TLS 1.2
        record.push(0x03).ok();
        
        // 加密数据
        let encrypted = self.encrypt_payload(data, &keys.client_write_key, &keys.client_write_iv)?;
        
        // 记录长度
        let length = encrypted.len() as u16;
        record.extend_from_slice(&length.to_be_bytes()).ok();
        
        // 加密的载荷
        record.extend_from_slice(&encrypted).ok();
        
        self.sequence_number += 1;
        
        Ok(record)
    }
    
    /// 解密应用数据
    pub fn decrypt_data(&mut self, record: &[u8]) -> Result<Vec<u8, 512>, &'static str> {
        if record.len() < 5 {
            return Err("Record too short");
        }
        
        if record[0] != TlsMessageType::ApplicationData as u8 {
            return Err("Not application data");
        }
        
        let length = u16::from_be_bytes([record[3], record[4]]) as usize;
        
        if record.len() < 5 + length {
            return Err("Incomplete record");
        }
        
        let keys = self.keys.as_ref().ok_or("Keys not available")?;
        let encrypted_data = &record[5..5+length];
        
        self.decrypt_payload(encrypted_data, &keys.server_write_key, &keys.server_write_iv)
    }
    
    fn generate_client_random(&mut self) {
        // 在实际实现中应使用安全的随机数生成器
        use core::ptr;
        unsafe {
            ptr::write_volatile(&mut self.session.client_random[0], 0x01);
        }
        
        // 填充时间戳和随机数
        let timestamp = 0x12345678u32; // 应使用实际时间戳
        self.session.client_random[0..4].copy_from_slice(&timestamp.to_be_bytes());
        
        // 其余28字节应为随机数
        for i in 4..32 {
            self.session.client_random[i] = (i as u8).wrapping_mul(17).wrapping_add(42);
        }
    }
    
    fn build_client_hello(&self) -> Result<Vec<u8, 256>, &'static str> {
        let mut hello = Vec::new();
        
        // TLS记录头
        hello.push(TlsMessageType::ClientHello as u8).ok();
        hello.push(0x03).ok(); // TLS 1.2
        hello.push(0x03).ok();
        
        // 握手消息头 (长度稍后填充)
        let length_pos = hello.len();
        hello.push(0).ok(); // 长度高字节
        hello.push(0).ok(); // 长度低字节
        
        // ClientHello消息
        hello.push(TlsMessageType::ClientHello as u8).ok();
        
        // 消息长度 (稍后填充)
        let msg_length_pos = hello.len();
        hello.push(0).ok();
        hello.push(0).ok();
        hello.push(0).ok();
        
        // 协议版本
        hello.push(0x03).ok(); // TLS 1.2
        hello.push(0x03).ok();
        
        // 客户端随机数
        hello.extend_from_slice(&self.session.client_random).ok();
        
        // 会话ID长度 (0)
        hello.push(0).ok();
        
        // 密码套件长度
        hello.push(0).ok();
        hello.push(2).ok();
        
        // 密码套件 (TLS_RSA_WITH_AES_128_CBC_SHA)
        hello.push(0x00).ok();
        hello.push(0x2F).ok();
        
        // 压缩方法长度
        hello.push(1).ok();
        
        // 压缩方法 (null)
        hello.push(0).ok();
        
        // 填充长度字段
        let total_length = hello.len() - 5;
        let msg_length = hello.len() - msg_length_pos - 3;
        
        hello[length_pos] = ((total_length >> 8) & 0xFF) as u8;
        hello[length_pos + 1] = (total_length & 0xFF) as u8;
        
        hello[msg_length_pos] = ((msg_length >> 16) & 0xFF) as u8;
        hello[msg_length_pos + 1] = ((msg_length >> 8) & 0xFF) as u8;
        hello[msg_length_pos + 2] = (msg_length & 0xFF) as u8;
        
        Ok(hello)
    }
    
    fn process_server_hello(&mut self, payload: &[u8]) -> Result<Option<Vec<u8, 256>>, &'static str> {
        if payload.len() < 38 {
            return Err("ServerHello too short");
        }
        
        // 提取服务器随机数
        self.session.server_random.copy_from_slice(&payload[2..34]);
        
        // 提取会话ID
        let session_id_len = payload[34] as usize;
        if payload.len() < 35 + session_id_len + 3 {
            return Err("ServerHello malformed");
        }
        
        self.session.session_id.clear();
        self.session.session_id.extend_from_slice(&payload[35..35+session_id_len]).ok();
        
        // 提取密码套件
        let cipher_suite_pos = 35 + session_id_len;
        self.session.cipher_suite = u16::from_be_bytes([
            payload[cipher_suite_pos],
            payload[cipher_suite_pos + 1]
        ]);
        
        // 提取压缩方法
        self.session.compression_method = payload[cipher_suite_pos + 2];
        
        self.state = TlsHandshakeState::ServerHelloReceived;
        
        Ok(None)
    }
    
    fn process_certificate(&mut self, _payload: &[u8]) -> Result<Option<Vec<u8, 256>>, &'static str> {
        // 在实际实现中应验证服务器证书
        self.state = TlsHandshakeState::CertificateReceived;
        Ok(None)
    }
    
    fn process_server_key_exchange(&mut self, _payload: &[u8]) -> Result<Option<Vec<u8, 256>>, &'static str> {
        self.state = TlsHandshakeState::KeyExchangeReceived;
        Ok(None)
    }
    
    fn process_server_hello_done(&mut self, _payload: &[u8]) -> Result<Option<Vec<u8, 256>>, &'static str> {
        self.state = TlsHandshakeState::ServerHelloDoneReceived;
        
        // 发送ClientKeyExchange
        let client_key_exchange = self.build_client_key_exchange()?;
        
        self.state = TlsHandshakeState::ClientKeyExchangeSent;
        
        Ok(Some(client_key_exchange))
    }
    
    fn process_finished(&mut self, _payload: &[u8]) -> Result<Option<Vec<u8, 256>>, &'static str> {
        self.state = TlsHandshakeState::HandshakeComplete;
        Ok(None)
    }
    
    fn build_client_key_exchange(&mut self) -> Result<Vec<u8, 256>, &'static str> {
        // 生成预主密钥
        let pre_master_secret = [0x03, 0x03]; // TLS 1.2版本
        let mut pms = [0u8; 48];
        pms[0..2].copy_from_slice(&pre_master_secret);
        
        // 其余46字节应为随机数
        for i in 2..48 {
            pms[i] = (i as u8).wrapping_mul(23).wrapping_add(67);
        }
        
        // 生成主密钥
        self.session.generate_master_secret(&pms);
        
        // 派生密钥
        self.keys = Some(self.session.derive_keys());
        
        // 构建ClientKeyExchange消息 (简化版本)
        let mut message = Vec::new();
        message.push(TlsMessageType::ClientKeyExchange as u8).ok();
        message.push(0x03).ok(); // TLS 1.2
        message.push(0x03).ok();
        
        // 消息长度 (简化)
        message.push(0).ok();
        message.push(50).ok();
        
        // 握手消息
        message.push(TlsMessageType::ClientKeyExchange as u8).ok();
        message.push(0).ok();
        message.push(0).ok();
        message.push(46).ok();
        
        // 加密的预主密钥 (简化)
        message.extend_from_slice(&pms).ok();
        
        Ok(message)
    }
    
    fn encrypt_payload(&self, data: &[u8], key: &[u8; 16], iv: &[u8; 16]) -> Result<Vec<u8, 512>, &'static str> {
        // 简化的AES-CBC加密
        use aes::cipher::{BlockEncrypt, KeyInit};
        
        let cipher = Aes128::new_from_slice(key).map_err(|_| "Invalid key")?;
        
        // 添加PKCS#7填充
        let mut padded_data = Vec::new();
        padded_data.extend_from_slice(data).ok();
        
        let padding_len = 16 - (data.len() % 16);
        for _ in 0..padding_len {
            padded_data.push(padding_len as u8).ok();
        }
        
        // CBC加密
        let mut encrypted = Vec::new();
        let mut prev_block = *iv;
        
        for chunk in padded_data.chunks(16) {
            let mut block = [0u8; 16];
            block.copy_from_slice(chunk);
            
            // XOR with previous block
            for i in 0..16 {
                block[i] ^= prev_block[i];
            }
            
            // Encrypt
            let mut block_array = block.into();
            cipher.encrypt_block(&mut block_array);
            let encrypted_block: [u8; 16] = block_array.into();
            
            encrypted.extend_from_slice(&encrypted_block).ok();
            prev_block = encrypted_block;
        }
        
        Ok(encrypted)
    }
    
    fn decrypt_payload(&self, data: &[u8], key: &[u8; 16], iv: &[u8; 16]) -> Result<Vec<u8, 512>, &'static str> {
        // 简化的AES-CBC解密
        use aes::cipher::{BlockDecrypt, KeyInit};
        
        if data.len() % 16 != 0 {
            return Err("Invalid ciphertext length");
        }
        
        let cipher = Aes128::new_from_slice(key).map_err(|_| "Invalid key")?;
        
        let mut decrypted = Vec::new();
        let mut prev_block = *iv;
        
        for chunk in data.chunks(16) {
            let mut block = [0u8; 16];
            block.copy_from_slice(chunk);
            
            let current_cipher = block;
            
            // Decrypt
            let mut block_array = block.into();
            cipher.decrypt_block(&mut block_array);
            let mut decrypted_block: [u8; 16] = block_array.into();
            
            // XOR with previous block
            for i in 0..16 {
                decrypted_block[i] ^= prev_block[i];
            }
            
            decrypted.extend_from_slice(&decrypted_block).ok();
            prev_block = current_cipher;
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
    
    pub fn get_state(&self) -> TlsHandshakeState {
        self.state
    }
    
    pub fn is_handshake_complete(&self) -> bool {
        self.state == TlsHandshakeState::HandshakeComplete
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_tls_session() {
        let mut session = TlsSession::new();
        let pre_master_secret = [0u8; 48];
        
        session.generate_master_secret(&pre_master_secret);
        let keys = session.derive_keys();
        
        // 验证密钥不全为零
        assert!(keys.client_write_key.iter().any(|&x| x != 0));
        assert!(keys.server_write_key.iter().any(|&x| x != 0));
    }
    
    #[test]
    fn test_tls_client_handshake() {
        let mut client = TlsClient::new();
        
        assert_eq!(client.get_state(), TlsHandshakeState::Start);
        
        let client_hello = client.start_handshake().unwrap();
        assert_eq!(client.get_state(), TlsHandshakeState::ClientHelloSent);
        assert!(!client_hello.is_empty());
    }
}
```

## 安全认证协议

### 基于挑战-响应的认证

```rust
use sha2::{Sha256, Digest};
use hmac::{Hmac, Mac};
use heapless::{String, Vec};

type HmacSha256 = Hmac<Sha256>;

/// 认证状态
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AuthState {
    Idle,
    ChallengeSent,
    ResponseReceived,
    Authenticated,
    Failed,
}

/// 认证协议
pub struct ChallengeResponseAuth {
    state: AuthState,
    challenge: [u8; 32],
    shared_key: [u8; 32],
    session_key: Option<[u8; 32]>,
    nonce_counter: u32,
}

impl ChallengeResponseAuth {
    pub fn new(shared_key: [u8; 32]) -> Self {
        Self {
            state: AuthState::Idle,
            challenge: [0u8; 32],
            shared_key,
            session_key: None,
            nonce_counter: 0,
        }
    }
    
    /// 生成认证挑战
    pub fn generate_challenge(&mut self) -> Result<Vec<u8, 64>, &'static str> {
        if self.state != AuthState::Idle {
            return Err("Authentication in progress");
        }
        
        // 生成随机挑战
        self.generate_random_challenge();
        
        // 构建挑战消息
        let mut message = Vec::new();
        message.push(0x01).ok(); // 挑战消息类型
        message.extend_from_slice(&self.challenge).ok();
        
        // 添加时间戳
        let timestamp = self.get_timestamp();
        message.extend_from_slice(&timestamp.to_be_bytes()).ok();
        
        self.state = AuthState::ChallengeSent;
        
        Ok(message)
    }
    
    /// 处理认证响应
    pub fn process_response(&mut self, response: &[u8]) -> Result<bool, &'static str> {
        if self.state != AuthState::ChallengeSent {
            return Err("No challenge sent");
        }
        
        if response.len() < 33 {
            return Err("Response too short");
        }
        
        if response[0] != 0x02 {
            return Err("Invalid response type");
        }
        
        let received_hash = &response[1..33];
        let expected_hash = self.calculate_expected_response();
        
        if received_hash == expected_hash {
            self.state = AuthState::Authenticated;
            self.generate_session_key();
            Ok(true)
        } else {
            self.state = AuthState::Failed;
            Ok(false)
        }
    }
    
    /// 生成认证响应 (客户端使用)
    pub fn generate_response(&self, challenge_msg: &[u8]) -> Result<Vec<u8, 64>, &'static str> {
        if challenge_msg.len() < 41 {
            return Err("Challenge message too short");
        }
        
        if challenge_msg[0] != 0x01 {
            return Err("Invalid challenge type");
        }
        
        let challenge = &challenge_msg[1..33];
        let timestamp_bytes = &challenge_msg[33..41];
        let timestamp = u64::from_be_bytes([
            timestamp_bytes[0], timestamp_bytes[1], timestamp_bytes[2], timestamp_bytes[3],
            timestamp_bytes[4], timestamp_bytes[5], timestamp_bytes[6], timestamp_bytes[7],
        ]);
        
        // 验证时间戳 (防重放攻击)
        let current_time = self.get_timestamp();
        if current_time.saturating_sub(timestamp) > 30 {
            return Err("Challenge expired");
        }
        
        // 计算响应
        let response_hash = self.calculate_response_hash(challenge, timestamp);
        
        let mut response = Vec::new();
        response.push(0x02).ok(); // 响应消息类型
        response.extend_from_slice(&response_hash).ok();
        
        Ok(response)
    }
    
    /// 加密消息
    pub fn encrypt_message(&mut self, data: &[u8]) -> Result<Vec<u8, 512>, &'static str> {
        if self.state != AuthState::Authenticated {
            return Err("Not authenticated");
        }
        
        let session_key = self.session_key.ok_or("No session key")?;
        
        // 生成nonce
        self.nonce_counter += 1;
        let nonce = self.nonce_counter.to_be_bytes();
        
        // 计算HMAC
        let mut mac = HmacSha256::new_from_slice(&session_key)
            .map_err(|_| "Invalid session key")?;
        mac.update(&nonce);
        mac.update(data);
        let tag = mac.finalize().into_bytes();
        
        // 构建加密消息
        let mut encrypted = Vec::new();
        encrypted.push(0x03).ok(); // 加密消息类型
        encrypted.extend_from_slice(&nonce).ok();
        encrypted.extend_from_slice(data).ok();
        encrypted.extend_from_slice(&tag[..16]).ok(); // 截断到16字节
        
        Ok(encrypted)
    }
    
    /// 解密消息
    pub fn decrypt_message(&self, encrypted: &[u8]) -> Result<Vec<u8, 512>, &'static str> {
        if self.state != AuthState::Authenticated {
            return Err("Not authenticated");
        }
        
        if encrypted.len() < 21 {
            return Err("Encrypted message too short");
        }
        
        if encrypted[0] != 0x03 {
            return Err("Invalid encrypted message type");
        }
        
        let session_key = self.session_key.ok_or("No session key")?;
        
        let nonce = &encrypted[1..5];
        let data_len = encrypted.len() - 21;
        let data = &encrypted[5..5+data_len];
        let received_tag = &encrypted[5+data_len..];
        
        // 验证HMAC
        let mut mac = HmacSha256::new_from_slice(&session_key)
            .map_err(|_| "Invalid session key")?;
        mac.update(nonce);
        mac.update(data);
        let expected_tag = mac.finalize().into_bytes();
        
        if received_tag != &expected_tag[..16] {
            return Err("Authentication tag mismatch");
        }
        
        let mut decrypted = Vec::new();
        decrypted.extend_from_slice(data).ok();
        
        Ok(decrypted)
    }
    
    fn generate_random_challenge(&mut self) {
        // 在实际实现中应使用安全的随机数生成器
        for i in 0..32 {
            self.challenge[i] = (i as u8).wrapping_mul(31).wrapping_add(73);
        }
    }
    
    fn calculate_expected_response(&self) -> [u8; 32] {
        let timestamp = self.get_timestamp();
        self.calculate_response_hash(&self.challenge, timestamp)
    }
    
    fn calculate_response_hash(&self, challenge: &[u8], timestamp: u64) -> [u8; 32] {
        let mut mac = HmacSha256::new_from_slice(&self.shared_key)
            .expect("Valid key length");
        
        mac.update(challenge);
        mac.update(&timestamp.to_be_bytes());
        
        let result = mac.finalize().into_bytes();
        let mut hash = [0u8; 32];
        hash.copy_from_slice(&result);
        hash
    }
    
    fn generate_session_key(&mut self) {
        let mut hasher = Sha256::new();
        hasher.update(&self.shared_key);
        hasher.update(&self.challenge);
        hasher.update(&self.nonce_counter.to_be_bytes());
        
        let result = hasher.finalize();
        let mut key = [0u8; 32];
        key.copy_from_slice(&result);
        
        self.session_key = Some(key);
    }
    
    fn get_timestamp(&self) -> u64 {
        // 在实际实现中应使用真实的时间戳
        12345678
    }
    
    pub fn get_state(&self) -> AuthState {
        self.state
    }
    
    pub fn is_authenticated(&self) -> bool {
        self.state == AuthState::Authenticated
    }
    
    pub fn reset(&mut self) {
        self.state = AuthState::Idle;
        self.challenge = [0u8; 32];
        self.session_key = None;
        self.nonce_counter = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_challenge_response_auth() {
        let shared_key = [0x42u8; 32];
        let mut server = ChallengeResponseAuth::new(shared_key);
        let client = ChallengeResponseAuth::new(shared_key);
        
        // 服务器生成挑战
        let challenge = server.generate_challenge().unwrap();
        assert_eq!(server.get_state(), AuthState::ChallengeSent);
        
        // 客户端生成响应
        let response = client.generate_response(&challenge).unwrap();
        
        // 服务器验证响应
        let is_valid = server.process_response(&response).unwrap();
        assert!(is_valid);
        assert!(server.is_authenticated());
    }
    
    #[test]
    fn test_encrypted_communication() {
        let shared_key = [0x42u8; 32];
        let mut server = ChallengeResponseAuth::new(shared_key);
        let client = ChallengeResponseAuth::new(shared_key);
        
        // 完成认证过程
        let challenge = server.generate_challenge().unwrap();
        let response = client.generate_response(&challenge).unwrap();
        server.process_response(&response).unwrap();
        
        // 加密消息
        let message = b"Hello, secure world!";
        let encrypted = server.encrypt_message(message).unwrap();
        
        // 解密消息
        let decrypted = server.decrypt_message(&encrypted).unwrap();
        assert_eq!(message, decrypted.as_slice());
    }
}
```

## 密钥管理

### 密钥派生和存储

```rust
use heapless::{FnvIndexMap, Vec};
use sha2::{Sha256, Digest};

/// 密钥类型
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum KeyType {
    Master,
    Session,
    Encryption,
    Authentication,
    Signing,
}

/// 密钥信息
#[derive(Debug, Clone)]
pub struct KeyInfo {
    pub key_id: u32,
    pub key_type: KeyType,
    pub key_data: Vec<u8, 64>,
    pub created_at: u64,
    pub expires_at: Option<u64>,
    pub usage_count: u32,
    pub max_usage: Option<u32>,
}

impl KeyInfo {
    pub fn new(
        key_id: u32,
        key_type: KeyType,
        key_data: &[u8],
        expires_at: Option<u64>
    ) -> Result<Self, &'static str> {
        if key_data.len() > 64 {
            return Err("Key too large");
        }
        
        let mut data = Vec::new();
        data.extend_from_slice(key_data).map_err(|_| "Key data too large")?;
        
        Ok(Self {
            key_id,
            key_type,
            key_data: data,
            created_at: Self::get_current_time(),
            expires_at,
            usage_count: 0,
            max_usage: None,
        })
    }
    
    pub fn is_expired(&self) -> bool {
        if let Some(expires_at) = self.expires_at {
            Self::get_current_time() > expires_at
        } else {
            false
        }
    }
    
    pub fn is_usage_exceeded(&self) -> bool {
        if let Some(max_usage) = self.max_usage {
            self.usage_count >= max_usage
        } else {
            false
        }
    }
    
    pub fn increment_usage(&mut self) {
        self.usage_count += 1;
    }
    
    fn get_current_time() -> u64 {
        // 在实际实现中应使用真实的时间戳
        12345678
    }
}

/// 密钥管理器
pub struct KeyManager {
    keys: FnvIndexMap<u32, KeyInfo, 16>,
    next_key_id: u32,
    master_key: Option<[u8; 32]>,
}

impl KeyManager {
    pub fn new() -> Self {
        Self {
            keys: FnvIndexMap::new(),
            next_key_id: 1,
            master_key: None,
        }
    }
    
    /// 设置主密钥
    pub fn set_master_key(&mut self, master_key: [u8; 32]) -> Result<(), &'static str> {
        self.master_key = Some(master_key);
        
        // 存储主密钥信息
        let key_info = KeyInfo::new(0, KeyType::Master, &master_key, None)?;
        self.keys.insert(0, key_info).map_err(|_| "Failed to store master key")?;
        
        Ok(())
    }
    
    /// 派生密钥
    pub fn derive_key(
        &mut self,
        key_type: KeyType,
        context: &[u8],
        length: usize
    ) -> Result<u32, &'static str> {
        let master_key = self.master_key.ok_or("No master key set")?;
        
        if length > 64 {
            return Err("Key length too large");
        }
        
        // 使用HKDF派生密钥
        let derived_key = self.hkdf_expand(&master_key, context, length)?;
        
        let key_id = self.next_key_id;
        self.next_key_id += 1;
        
        // 设置密钥过期时间 (1小时)
        let expires_at = Some(KeyInfo::get_current_time() + 3600);
        
        let key_info = KeyInfo::new(key_id, key_type, &derived_key, expires_at)?;
        
        self.keys.insert(key_id, key_info).map_err(|_| "Failed to store derived key")?;
        
        Ok(key_id)
    }
    
    /// 获取密钥
    pub fn get_key(&mut self, key_id: u32) -> Result<&[u8], &'static str> {
        let key_info = self.keys.get_mut(&key_id).ok_or("Key not found")?;
        
        if key_info.is_expired() {
            return Err("Key expired");
        }
        
        if key_info.is_usage_exceeded() {
            return Err("Key usage exceeded");
        }
        
        key_info.increment_usage();
        
        Ok(&key_info.key_data)
    }
    
    /// 删除密钥
    pub fn delete_key(&mut self, key_id: u32) -> Result<(), &'static str> {
        if key_id == 0 {
            return Err("Cannot delete master key");
        }
        
        self.keys.remove(&key_id).ok_or("Key not found")?;
        Ok(())
    }
    
    /// 清理过期密钥
    pub fn cleanup_expired_keys(&mut self) {
        let mut expired_keys = Vec::<u32, 16>::new();
        
        for (&key_id, key_info) in &self.keys {
            if key_id != 0 && (key_info.is_expired() || key_info.is_usage_exceeded()) {
                expired_keys.push(key_id).ok();
            }
        }
        
        for key_id in &expired_keys {
            self.keys.remove(key_id);
        }
    }
    
    /// 轮换密钥
    pub fn rotate_key(&mut self, old_key_id: u32, context: &[u8]) -> Result<u32, &'static str> {
        let old_key_info = self.keys.get(&old_key_id).ok_or("Old key not found")?;
        let key_type = old_key_info.key_type;
        let key_length = old_key_info.key_data.len();
        
        // 派生新密钥
        let new_key_id = self.derive_key(key_type, context, key_length)?;
        
        // 标记旧密钥为即将过期
        if let Some(old_key) = self.keys.get_mut(&old_key_id) {
            old_key.expires_at = Some(KeyInfo::get_current_time() + 300); // 5分钟后过期
        }
        
        Ok(new_key_id)
    }
    
    /// 获取密钥信息
    pub fn get_key_info(&self, key_id: u32) -> Option<&KeyInfo> {
        self.keys.get(&key_id)
    }
    
    /// 列出所有密钥
    pub fn list_keys(&self) -> Vec<u32, 16> {
        let mut key_ids = Vec::new();
        
        for &key_id in self.keys.keys() {
            key_ids.push(key_id).ok();
        }
        
        key_ids
    }
    
    fn hkdf_expand(&self, key: &[u8], info: &[u8], length: usize) -> Result<Vec<u8, 64>, &'static str> {
        // 简化的HKDF实现
        let mut hasher = Sha256::new();
        hasher.update(key);
        hasher.update(info);
        hasher.update(&[0x01]); // Counter
        
        let hash = hasher.finalize();
        
        let mut output = Vec::new();
        let copy_len = length.min(32).min(64);
        output.extend_from_slice(&hash[..copy_len]).map_err(|_| "Output too large")?;
        
        Ok(output)
    }
    
    pub fn get_key_count(&self) -> usize {
        self.keys.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_key_manager() {
        let mut manager = KeyManager::new();
        let master_key = [0x42u8; 32];
        
        // 设置主密钥
        manager