# 生产部署

## 概述

生产部署是嵌入式系统开发的最后阶段，需要确保系统在实际环境中稳定可靠地运行。本文档介绍生产部署的各个方面，包括部署策略、环境配置、质量保证和发布管理。

## 部署策略

### 蓝绿部署

```rust
// 部署管理器
use embedded_hal::digital::v2::{InputPin, OutputPin};
use heapless::Vec;

#[derive(Debug, Clone, Copy)]
pub enum DeploymentSlot {
    Blue,
    Green,
}

#[derive(Debug, Clone, Copy)]
pub enum DeploymentStatus {
    Inactive,
    Active,
    Deploying,
    Failed,
}

pub struct BlueGreenDeployment<P1, P2, P3> 
where
    P1: OutputPin,
    P2: OutputPin,
    P3: InputPin,
{
    blue_enable: P1,
    green_enable: P2,
    health_check: P3,
    current_slot: DeploymentSlot,
    blue_status: DeploymentStatus,
    green_status: DeploymentStatus,
}

impl<P1, P2, P3> BlueGreenDeployment<P1, P2, P3>
where
    P1: OutputPin,
    P2: OutputPin,
    P3: InputPin,
{
    pub fn new(blue_enable: P1, green_enable: P2, health_check: P3) -> Self {
        Self {
            blue_enable,
            green_enable,
            health_check,
            current_slot: DeploymentSlot::Blue,
            blue_status: DeploymentStatus::Active,
            green_status: DeploymentStatus::Inactive,
        }
    }

    pub fn deploy_to_inactive_slot(&mut self, firmware: &[u8]) -> Result<(), DeploymentError> {
        let target_slot = match self.current_slot {
            DeploymentSlot::Blue => DeploymentSlot::Green,
            DeploymentSlot::Green => DeploymentSlot::Blue,
        };

        // 设置目标槽状态为部署中
        match target_slot {
            DeploymentSlot::Blue => self.blue_status = DeploymentStatus::Deploying,
            DeploymentSlot::Green => self.green_status = DeploymentStatus::Deploying,
        }

        // 部署固件
        self.flash_firmware(target_slot, firmware)?;

        // 健康检查
        if self.health_check_slot(target_slot)? {
            match target_slot {
                DeploymentSlot::Blue => self.blue_status = DeploymentStatus::Inactive,
                DeploymentSlot::Green => self.green_status = DeploymentStatus::Inactive,
            }
            Ok(())
        } else {
            match target_slot {
                DeploymentSlot::Blue => self.blue_status = DeploymentStatus::Failed,
                DeploymentSlot::Green => self.green_status = DeploymentStatus::Failed,
            }
            Err(DeploymentError::HealthCheckFailed)
        }
    }

    pub fn switch_active_slot(&mut self) -> Result<(), DeploymentError> {
        let new_slot = match self.current_slot {
            DeploymentSlot::Blue => DeploymentSlot::Green,
            DeploymentSlot::Green => DeploymentSlot::Blue,
        };

        // 检查新槽是否准备就绪
        let new_slot_status = match new_slot {
            DeploymentSlot::Blue => self.blue_status,
            DeploymentSlot::Green => self.green_status,
        };

        if !matches!(new_slot_status, DeploymentStatus::Inactive) {
            return Err(DeploymentError::SlotNotReady);
        }

        // 切换槽
        match new_slot {
            DeploymentSlot::Blue => {
                self.blue_enable.set_high().map_err(|_| DeploymentError::HardwareError)?;
                self.green_enable.set_low().map_err(|_| DeploymentError::HardwareError)?;
                self.blue_status = DeploymentStatus::Active;
                self.green_status = DeploymentStatus::Inactive;
            }
            DeploymentSlot::Green => {
                self.green_enable.set_high().map_err(|_| DeploymentError::HardwareError)?;
                self.blue_enable.set_low().map_err(|_| DeploymentError::HardwareError)?;
                self.green_status = DeploymentStatus::Active;
                self.blue_status = DeploymentStatus::Inactive;
            }
        }

        self.current_slot = new_slot;
        Ok(())
    }

    fn flash_firmware(&mut self, slot: DeploymentSlot, firmware: &[u8]) -> Result<(), DeploymentError> {
        // 模拟固件刷写
        // 实际实现中会调用flash驱动
        if firmware.len() > 0 {
            Ok(())
        } else {
            Err(DeploymentError::InvalidFirmware)
        }
    }

    fn health_check_slot(&mut self, slot: DeploymentSlot) -> Result<bool, DeploymentError> {
        // 模拟健康检查
        // 实际实现中会检查系统状态
        match self.health_check.is_high() {
            Ok(high) => Ok(high),
            Err(_) => Err(DeploymentError::HealthCheckFailed),
        }
    }
}

#[derive(Debug)]
pub enum DeploymentError {
    HardwareError,
    InvalidFirmware,
    HealthCheckFailed,
    SlotNotReady,
}
```

### 滚动部署

```rust
// 滚动部署管理器
use heapless::Vec;

pub struct RollingDeployment {
    nodes: Vec<DeploymentNode, 16>,
    batch_size: usize,
    current_batch: usize,
}

#[derive(Debug, Clone)]
pub struct DeploymentNode {
    id: u8,
    address: u32,
    status: NodeStatus,
    version: u32,
}

#[derive(Debug, Clone, Copy)]
pub enum NodeStatus {
    Ready,
    Updating,
    Updated,
    Failed,
    Rollback,
}

impl RollingDeployment {
    pub fn new(batch_size: usize) -> Self {
        Self {
            nodes: Vec::new(),
            batch_size,
            current_batch: 0,
        }
    }

    pub fn add_node(&mut self, id: u8, address: u32) -> Result<(), ()> {
        let node = DeploymentNode {
            id,
            address,
            status: NodeStatus::Ready,
            version: 0,
        };
        self.nodes.push(node).map_err(|_| ())
    }

    pub fn deploy_version(&mut self, version: u32, firmware: &[u8]) -> Result<(), DeploymentError> {
        self.current_batch = 0;

        while self.current_batch * self.batch_size < self.nodes.len() {
            self.deploy_batch(version, firmware)?;
            self.current_batch += 1;
        }

        Ok(())
    }

    fn deploy_batch(&mut self, version: u32, firmware: &[u8]) -> Result<(), DeploymentError> {
        let start_idx = self.current_batch * self.batch_size;
        let end_idx = core::cmp::min(start_idx + self.batch_size, self.nodes.len());

        // 更新批次中的节点
        for i in start_idx..end_idx {
            if let Some(node) = self.nodes.get_mut(i) {
                node.status = NodeStatus::Updating;
                
                // 模拟部署过程
                if self.deploy_to_node(node, version, firmware)? {
                    node.status = NodeStatus::Updated;
                    node.version = version;
                } else {
                    node.status = NodeStatus::Failed;
                    return Err(DeploymentError::NodeUpdateFailed);
                }
            }
        }

        // 健康检查
        self.health_check_batch(start_idx, end_idx)?;

        Ok(())
    }

    fn deploy_to_node(&self, node: &DeploymentNode, version: u32, firmware: &[u8]) -> Result<bool, DeploymentError> {
        // 模拟网络部署
        // 实际实现中会通过网络发送固件
        if firmware.len() > 0 && node.address != 0 {
            Ok(true)
        } else {
            Ok(false)
        }
    }

    fn health_check_batch(&mut self, start_idx: usize, end_idx: usize) -> Result<(), DeploymentError> {
        for i in start_idx..end_idx {
            if let Some(node) = self.nodes.get_mut(i) {
                if matches!(node.status, NodeStatus::Updated) {
                    // 模拟健康检查
                    if !self.ping_node(node.address) {
                        node.status = NodeStatus::Failed;
                        return Err(DeploymentError::HealthCheckFailed);
                    }
                }
            }
        }
        Ok(())
    }

    fn ping_node(&self, address: u32) -> bool {
        // 模拟网络ping
        address != 0
    }

    pub fn rollback(&mut self, target_version: u32) -> Result<(), DeploymentError> {
        for node in &mut self.nodes {
            if node.version > target_version {
                node.status = NodeStatus::Rollback;
                // 实际实现中会执行回滚操作
                node.version = target_version;
                node.status = NodeStatus::Ready;
            }
        }
        Ok(())
    }
}
```

## 环境配置

### 配置管理

```rust
// 环境配置管理
use serde::{Deserialize, Serialize};
use heapless::{String, Vec};

#[derive(Debug, Serialize, Deserialize)]
pub struct ProductionConfig {
    pub environment: Environment,
    pub network: NetworkConfig,
    pub security: SecurityConfig,
    pub logging: LoggingConfig,
    pub monitoring: MonitoringConfig,
}

#[derive(Debug, Serialize, Deserialize)]
pub enum Environment {
    Development,
    Staging,
    Production,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct NetworkConfig {
    pub ip_address: String<16>,
    pub subnet_mask: String<16>,
    pub gateway: String<16>,
    pub dns_servers: Vec<String<16>, 4>,
    pub ntp_server: String<32>,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct SecurityConfig {
    pub enable_encryption: bool,
    pub certificate_path: String<64>,
    pub key_path: String<64>,
    pub allowed_origins: Vec<String<32>, 8>,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct LoggingConfig {
    pub level: LogLevel,
    pub output: LogOutput,
    pub max_file_size: u32,
    pub rotation_count: u8,
}

#[derive(Debug, Serialize, Deserialize)]
pub enum LogLevel {
    Error,
    Warn,
    Info,
    Debug,
    Trace,
}

#[derive(Debug, Serialize, Deserialize)]
pub enum LogOutput {
    Console,
    File,
    Network,
    Both,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct MonitoringConfig {
    pub enable_metrics: bool,
    pub metrics_endpoint: String<64>,
    pub health_check_interval: u32,
    pub alert_thresholds: AlertThresholds,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct AlertThresholds {
    pub cpu_usage: u8,
    pub memory_usage: u8,
    pub temperature: i16,
    pub error_rate: f32,
}

pub struct ConfigManager {
    config: ProductionConfig,
}

impl ConfigManager {
    pub fn load_from_flash() -> Result<Self, ConfigError> {
        // 模拟从Flash加载配置
        let config = ProductionConfig {
            environment: Environment::Production,
            network: NetworkConfig {
                ip_address: String::from("192.168.1.100"),
                subnet_mask: String::from("255.255.255.0"),
                gateway: String::from("192.168.1.1"),
                dns_servers: {
                    let mut dns = Vec::new();
                    dns.push(String::from("8.8.8.8")).ok();
                    dns.push(String::from("8.8.4.4")).ok();
                    dns
                },
                ntp_server: String::from("pool.ntp.org"),
            },
            security: SecurityConfig {
                enable_encryption: true,
                certificate_path: String::from("/certs/device.crt"),
                key_path: String::from("/certs/device.key"),
                allowed_origins: Vec::new(),
            },
            logging: LoggingConfig {
                level: LogLevel::Info,
                output: LogOutput::Both,
                max_file_size: 1024 * 1024, // 1MB
                rotation_count: 5,
            },
            monitoring: MonitoringConfig {
                enable_metrics: true,
                metrics_endpoint: String::from("http://monitor.example.com/metrics"),
                health_check_interval: 30, // 30秒
                alert_thresholds: AlertThresholds {
                    cpu_usage: 80,
                    memory_usage: 85,
                    temperature: 70,
                    error_rate: 0.05,
                },
            },
        };

        Ok(Self { config })
    }

    pub fn get_config(&self) -> &ProductionConfig {
        &self.config
    }

    pub fn update_config(&mut self, new_config: ProductionConfig) -> Result<(), ConfigError> {
        // 验证配置
        self.validate_config(&new_config)?;
        
        // 保存到Flash
        self.save_to_flash(&new_config)?;
        
        self.config = new_config;
        Ok(())
    }

    fn validate_config(&self, config: &ProductionConfig) -> Result<(), ConfigError> {
        // 验证网络配置
        if config.network.ip_address.is_empty() {
            return Err(ConfigError::InvalidNetworkConfig);
        }

        // 验证安全配置
        if config.security.enable_encryption && config.security.certificate_path.is_empty() {
            return Err(ConfigError::InvalidSecurityConfig);
        }

        Ok(())
    }

    fn save_to_flash(&self, config: &ProductionConfig) -> Result<(), ConfigError> {
        // 模拟保存到Flash
        // 实际实现中会序列化配置并写入Flash
        Ok(())
    }
}

#[derive(Debug)]
pub enum ConfigError {
    LoadFailed,
    SaveFailed,
    InvalidNetworkConfig,
    InvalidSecurityConfig,
    SerializationError,
}
```

## 质量保证

### 自动化测试

```rust
// 生产测试套件
use heapless::Vec;

pub struct ProductionTestSuite {
    tests: Vec<ProductionTest, 32>,
    results: Vec<TestResult, 32>,
}

#[derive(Debug, Clone)]
pub struct ProductionTest {
    pub name: &'static str,
    pub test_fn: fn() -> TestResult,
    pub timeout_ms: u32,
    pub critical: bool,
}

#[derive(Debug, Clone, Copy)]
pub struct TestResult {
    pub passed: bool,
    pub duration_ms: u32,
    pub error_code: Option<u32>,
}

impl ProductionTestSuite {
    pub fn new() -> Self {
        Self {
            tests: Vec::new(),
            results: Vec::new(),
        }
    }

    pub fn add_test(&mut self, test: ProductionTest) -> Result<(), ()> {
        self.tests.push(test).map_err(|_| ())
    }

    pub fn run_all_tests(&mut self) -> ProductionTestReport {
        self.results.clear();
        let mut passed = 0;
        let mut failed = 0;
        let mut critical_failures = 0;

        for test in &self.tests {
            let start_time = self.get_timestamp();
            let result = (test.test_fn)();
            let end_time = self.get_timestamp();

            let test_result = TestResult {
                passed: result.passed,
                duration_ms: end_time - start_time,
                error_code: result.error_code,
            };

            if test_result.passed {
                passed += 1;
            } else {
                failed += 1;
                if test.critical {
                    critical_failures += 1;
                }
            }

            self.results.push(test_result).ok();
        }

        ProductionTestReport {
            total_tests: self.tests.len(),
            passed,
            failed,
            critical_failures,
            overall_passed: critical_failures == 0,
        }
    }

    fn get_timestamp(&self) -> u32 {
        // 模拟时间戳获取
        0
    }
}

#[derive(Debug)]
pub struct ProductionTestReport {
    pub total_tests: usize,
    pub passed: usize,
    pub failed: usize,
    pub critical_failures: usize,
    pub overall_passed: bool,
}

// 具体测试函数
pub fn test_hardware_interfaces() -> TestResult {
    // GPIO测试
    if !test_gpio() {
        return TestResult {
            passed: false,
            duration_ms: 0,
            error_code: Some(0x1001),
        };
    }

    // UART测试
    if !test_uart() {
        return TestResult {
            passed: false,
            duration_ms: 0,
            error_code: Some(0x1002),
        };
    }

    TestResult {
        passed: true,
        duration_ms: 0,
        error_code: None,
    }
}

pub fn test_network_connectivity() -> TestResult {
    // 网络连接测试
    if !ping_gateway() {
        return TestResult {
            passed: false,
            duration_ms: 0,
            error_code: Some(0x2001),
        };
    }

    TestResult {
        passed: true,
        duration_ms: 0,
        error_code: None,
    }
}

pub fn test_security_features() -> TestResult {
    // 加密功能测试
    if !test_encryption() {
        return TestResult {
            passed: false,
            duration_ms: 0,
            error_code: Some(0x3001),
        };
    }

    // 证书验证测试
    if !test_certificate_validation() {
        return TestResult {
            passed: false,
            duration_ms: 0,
            error_code: Some(0x3002),
        };
    }

    TestResult {
        passed: true,
        duration_ms: 0,
        error_code: None,
    }
}

// 辅助测试函数
fn test_gpio() -> bool {
    // 模拟GPIO测试
    true
}

fn test_uart() -> bool {
    // 模拟UART测试
    true
}

fn ping_gateway() -> bool {
    // 模拟网关ping测试
    true
}

fn test_encryption() -> bool {
    // 模拟加密测试
    true
}

fn test_certificate_validation() -> bool {
    // 模拟证书验证测试
    true
}
```

## 发布管理

### 版本控制

```rust
// 版本管理系统
use serde::{Deserialize, Serialize};
use heapless::{String, Vec};

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct Version {
    pub major: u16,
    pub minor: u16,
    pub patch: u16,
    pub build: u32,
    pub pre_release: Option<String<16>>,
}

impl Version {
    pub fn new(major: u16, minor: u16, patch: u16) -> Self {
        Self {
            major,
            minor,
            patch,
            build: 0,
            pre_release: None,
        }
    }

    pub fn to_string(&self) -> String<32> {
        let mut version_str = String::new();
        version_str.push_str(&format!("{}.{}.{}", self.major, self.minor, self.patch)).ok();
        
        if self.build > 0 {
            version_str.push_str(&format!("+{}", self.build)).ok();
        }

        if let Some(ref pre) = self.pre_release {
            version_str.push('-').ok();
            version_str.push_str(pre).ok();
        }

        version_str
    }

    pub fn is_compatible(&self, other: &Version) -> bool {
        self.major == other.major
    }

    pub fn is_newer(&self, other: &Version) -> bool {
        if self.major != other.major {
            return self.major > other.major;
        }
        if self.minor != other.minor {
            return self.minor > other.minor;
        }
        if self.patch != other.patch {
            return self.patch > other.patch;
        }
        self.build > other.build
    }
}

#[derive(Debug, Serialize, Deserialize)]
pub struct ReleaseInfo {
    pub version: Version,
    pub release_date: u64,
    pub changelog: Vec<String<128>, 16>,
    pub checksum: String<64>,
    pub signature: String<128>,
    pub critical: bool,
}

pub struct ReleaseManager {
    current_version: Version,
    release_history: Vec<ReleaseInfo, 32>,
}

impl ReleaseManager {
    pub fn new(initial_version: Version) -> Self {
        Self {
            current_version: initial_version,
            release_history: Vec::new(),
        }
    }

    pub fn create_release(&mut self, version: Version, changelog: Vec<String<128>, 16>) -> Result<ReleaseInfo, ReleaseError> {
        // 验证版本号
        if !version.is_newer(&self.current_version) {
            return Err(ReleaseError::InvalidVersion);
        }

        // 生成校验和和签名
        let checksum = self.calculate_checksum(&version)?;
        let signature = self.generate_signature(&version, &checksum)?;

        let release = ReleaseInfo {
            version: version.clone(),
            release_date: self.get_timestamp(),
            changelog,
            checksum,
            signature,
            critical: false,
        };

        self.release_history.push(release.clone()).map_err(|_| ReleaseError::StorageFull)?;
        self.current_version = version;

        Ok(release)
    }

    pub fn get_current_version(&self) -> &Version {
        &self.current_version
    }

    pub fn get_release_history(&self) -> &Vec<ReleaseInfo, 32> {
        &self.release_history
    }

    pub fn validate_release(&self, release: &ReleaseInfo) -> Result<bool, ReleaseError> {
        // 验证校验和
        let expected_checksum = self.calculate_checksum(&release.version)?;
        if release.checksum != expected_checksum {
            return Ok(false);
        }

        // 验证签名
        if !self.verify_signature(&release.version, &release.checksum, &release.signature)? {
            return Ok(false);
        }

        Ok(true)
    }

    fn calculate_checksum(&self, version: &Version) -> Result<String<64>, ReleaseError> {
        // 模拟校验和计算
        let mut checksum = String::new();
        checksum.push_str("sha256:").ok();
        checksum.push_str(&format!("{:08x}", version.major as u32 + version.minor as u32 + version.patch as u32)).ok();
        Ok(checksum)
    }

    fn generate_signature(&self, version: &Version, checksum: &str) -> Result<String<128>, ReleaseError> {
        // 模拟数字签名生成
        let mut signature = String::new();
        signature.push_str("sig:").ok();
        signature.push_str(&format!("{:016x}", version.build)).ok();
        Ok(signature)
    }

    fn verify_signature(&self, version: &Version, checksum: &str, signature: &str) -> Result<bool, ReleaseError> {
        // 模拟签名验证
        let expected_signature = self.generate_signature(version, checksum)?;
        Ok(signature == expected_signature.as_str())
    }

    fn get_timestamp(&self) -> u64 {
        // 模拟时间戳获取
        1640995200 // 2022-01-01 00:00:00 UTC
    }
}

#[derive(Debug)]
pub enum ReleaseError {
    InvalidVersion,
    StorageFull,
    ChecksumError,
    SignatureError,
}
```

## 总结

生产部署是嵌入式系统开发的关键阶段，需要：

1. **部署策略**：选择合适的部署方式（蓝绿部署、滚动部署等）
2. **环境配置**：管理不同环境的配置参数
3. **质量保证**：实施全面的自动化测试
4. **发布管理**：建立完善的版本控制和发布流程

通过系统化的部署管理，可以确保嵌入式系统在生产环境中稳定可靠地运行。