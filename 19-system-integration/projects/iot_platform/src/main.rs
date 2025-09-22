use anyhow::{Context, Result};
use axum::{
    extract::{Path, Query, State},
    http::StatusCode,
    response::Json,
    routing::{get, post, put, delete},
    Router,
};
use chrono::{DateTime, Utc};
use dashmap::DashMap;
use serde::{Deserialize, Serialize};
use sqlx::{PgPool, Row};
use std::collections::HashMap;
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::{broadcast, RwLock};
use tower_http::cors::CorsLayer;
use tracing::{debug, error, info, warn};
use uuid::Uuid;

// 设备类型
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum DeviceType {
    Sensor,
    Actuator,
    Gateway,
    Camera,
    Controller,
    Custom(String),
}

// 设备状态
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum DeviceStatus {
    Online,
    Offline,
    Maintenance,
    Error,
    Unknown,
}

// 设备信息
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Device {
    pub id: Uuid,
    pub name: String,
    pub device_type: DeviceType,
    pub status: DeviceStatus,
    pub location: Option<String>,
    pub metadata: HashMap<String, String>,
    pub last_seen: DateTime<Utc>,
    pub created_at: DateTime<Utc>,
    pub updated_at: DateTime<Utc>,
}

// 传感器数据
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SensorData {
    pub id: Uuid,
    pub device_id: Uuid,
    pub sensor_type: String,
    pub value: f64,
    pub unit: String,
    pub quality: f32, // 数据质量 0-1
    pub timestamp: DateTime<Utc>,
    pub metadata: HashMap<String, String>,
}

// 数据点
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DataPoint {
    pub timestamp: DateTime<Utc>,
    pub value: f64,
    pub quality: f32,
}

// 时间序列数据
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TimeSeries {
    pub device_id: Uuid,
    pub sensor_type: String,
    pub data_points: Vec<DataPoint>,
    pub aggregation: Option<String>, // avg, sum, min, max
    pub interval: Option<Duration>,
}

// 规则类型
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum RuleType {
    Threshold,    // 阈值规则
    Trend,        // 趋势规则
    Anomaly,      // 异常检测
    Correlation,  // 关联规则
    Schedule,     // 定时规则
    Custom(String),
}

// 规则条件
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RuleCondition {
    pub field: String,
    pub operator: String, // >, <, ==, !=, contains, etc.
    pub value: String,
    pub logical_op: Option<String>, // AND, OR
}

// 规则动作
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RuleAction {
    pub action_type: String, // alert, control, webhook, email
    pub target: String,
    pub parameters: HashMap<String, String>,
}

// 业务规则
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Rule {
    pub id: Uuid,
    pub name: String,
    pub description: String,
    pub rule_type: RuleType,
    pub conditions: Vec<RuleCondition>,
    pub actions: Vec<RuleAction>,
    pub enabled: bool,
    pub priority: u8, // 1-10
    pub created_at: DateTime<Utc>,
    pub updated_at: DateTime<Utc>,
}

// 告警级别
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum AlertLevel {
    Info,
    Warning,
    Error,
    Critical,
}

// 告警信息
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Alert {
    pub id: Uuid,
    pub rule_id: Uuid,
    pub device_id: Option<Uuid>,
    pub level: AlertLevel,
    pub title: String,
    pub message: String,
    pub data: HashMap<String, String>,
    pub acknowledged: bool,
    pub resolved: bool,
    pub created_at: DateTime<Utc>,
    pub resolved_at: Option<DateTime<Utc>>,
}

// 设备管理器
pub struct DeviceManager {
    devices: Arc<DashMap<Uuid, Device>>,
    db_pool: PgPool,
    event_sender: broadcast::Sender<DeviceEvent>,
}

#[derive(Debug, Clone)]
pub enum DeviceEvent {
    Connected(Uuid),
    Disconnected(Uuid),
    DataReceived(Uuid, SensorData),
    StatusChanged(Uuid, DeviceStatus),
}

impl DeviceManager {
    pub fn new(db_pool: PgPool) -> Self {
        let (event_sender, _) = broadcast::channel(1000);
        
        Self {
            devices: Arc::new(DashMap::new()),
            db_pool,
            event_sender,
        }
    }

    pub async fn register_device(&self, mut device: Device) -> Result<()> {
        device.created_at = Utc::now();
        device.updated_at = Utc::now();
        
        // 保存到数据库
        sqlx::query!(
            r#"
            INSERT INTO devices (id, name, device_type, status, location, metadata, last_seen, created_at, updated_at)
            VALUES ($1, $2, $3, $4, $5, $6, $7, $8, $9)
            "#,
            device.id,
            device.name,
            serde_json::to_string(&device.device_type)?,
            serde_json::to_string(&device.status)?,
            device.location,
            serde_json::to_string(&device.metadata)?,
            device.last_seen,
            device.created_at,
            device.updated_at
        )
        .execute(&self.db_pool)
        .await?;
        
        // 缓存到内存
        self.devices.insert(device.id, device.clone());
        
        // 发送事件
        let _ = self.event_sender.send(DeviceEvent::Connected(device.id));
        
        info!("Device registered: {} ({})", device.name, device.id);
        Ok(())
    }

    pub async fn update_device_status(&self, device_id: Uuid, status: DeviceStatus) -> Result<()> {
        if let Some(mut device) = self.devices.get_mut(&device_id) {
            device.status = status.clone();
            device.last_seen = Utc::now();
            device.updated_at = Utc::now();
            
            // 更新数据库
            sqlx::query!(
                "UPDATE devices SET status = $1, last_seen = $2, updated_at = $3 WHERE id = $4",
                serde_json::to_string(&status)?,
                device.last_seen,
                device.updated_at,
                device_id
            )
            .execute(&self.db_pool)
            .await?;
            
            // 发送事件
            let _ = self.event_sender.send(DeviceEvent::StatusChanged(device_id, status));
        }
        
        Ok(())
    }

    pub fn get_device(&self, device_id: &Uuid) -> Option<Device> {
        self.devices.get(device_id).map(|d| d.clone())
    }

    pub fn list_devices(&self) -> Vec<Device> {
        self.devices.iter().map(|entry| entry.value().clone()).collect()
    }

    pub fn get_devices_by_type(&self, device_type: &DeviceType) -> Vec<Device> {
        self.devices
            .iter()
            .filter(|entry| &entry.value().device_type == device_type)
            .map(|entry| entry.value().clone())
            .collect()
    }

    pub fn get_online_devices(&self) -> Vec<Device> {
        self.devices
            .iter()
            .filter(|entry| entry.value().status == DeviceStatus::Online)
            .map(|entry| entry.value().clone())
            .collect()
    }

    pub fn subscribe_events(&self) -> broadcast::Receiver<DeviceEvent> {
        self.event_sender.subscribe()
    }
}

// 数据处理器
pub struct DataProcessor {
    db_pool: PgPool,
    redis_client: redis::Client,
    time_series_cache: Arc<DashMap<String, Vec<DataPoint>>>,
}

impl DataProcessor {
    pub fn new(db_pool: PgPool, redis_url: &str) -> Result<Self> {
        let redis_client = redis::Client::open(redis_url)?;
        
        Ok(Self {
            db_pool,
            redis_client,
            time_series_cache: Arc::new(DashMap::new()),
        })
    }

    pub async fn process_sensor_data(&self, data: SensorData) -> Result<()> {
        // 数据验证
        self.validate_data(&data)?;
        
        // 数据清洗
        let cleaned_data = self.clean_data(data).await?;
        
        // 存储到数据库
        self.store_data(&cleaned_data).await?;
        
        // 更新时间序列缓存
        self.update_time_series_cache(&cleaned_data).await?;
        
        // 实时数据推送
        self.push_real_time_data(&cleaned_data).await?;
        
        debug!("Processed sensor data: {:?}", cleaned_data.id);
        Ok(())
    }

    fn validate_data(&self, data: &SensorData) -> Result<()> {
        // 检查数据完整性
        if data.sensor_type.is_empty() {
            return Err(anyhow::anyhow!("Sensor type cannot be empty"));
        }
        
        // 检查数据质量
        if data.quality < 0.0 || data.quality > 1.0 {
            return Err(anyhow::anyhow!("Data quality must be between 0 and 1"));
        }
        
        // 检查时间戳
        let now = Utc::now();
        if data.timestamp > now + chrono::Duration::minutes(5) {
            return Err(anyhow::anyhow!("Timestamp is too far in the future"));
        }
        
        Ok(())
    }

    async fn clean_data(&self, mut data: SensorData) -> Result<SensorData> {
        // 数据范围检查
        match data.sensor_type.as_str() {
            "temperature" => {
                if data.value < -50.0 || data.value > 100.0 {
                    warn!("Temperature value out of range: {}", data.value);
                    data.quality *= 0.5; // 降低数据质量
                }
            }
            "humidity" => {
                if data.value < 0.0 || data.value > 100.0 {
                    warn!("Humidity value out of range: {}", data.value);
                    data.value = data.value.clamp(0.0, 100.0);
                }
            }
            "pressure" => {
                if data.value < 800.0 || data.value > 1200.0 {
                    warn!("Pressure value out of range: {}", data.value);
                    data.quality *= 0.7;
                }
            }
            _ => {} // 其他传感器类型的处理
        }
        
        // 异常值检测
        if let Some(historical_avg) = self.get_historical_average(&data.device_id, &data.sensor_type).await? {
            let deviation = (data.value - historical_avg).abs() / historical_avg;
            if deviation > 0.5 { // 偏差超过50%
                warn!("Anomalous value detected: {} (expected ~{})", data.value, historical_avg);
                data.quality *= 0.8;
            }
        }
        
        Ok(data)
    }

    async fn store_data(&self, data: &SensorData) -> Result<()> {
        sqlx::query!(
            r#"
            INSERT INTO sensor_data (id, device_id, sensor_type, value, unit, quality, timestamp, metadata)
            VALUES ($1, $2, $3, $4, $5, $6, $7, $8)
            "#,
            data.id,
            data.device_id,
            data.sensor_type,
            data.value,
            data.unit,
            data.quality,
            data.timestamp,
            serde_json::to_string(&data.metadata)?
        )
        .execute(&self.db_pool)
        .await?;
        
        Ok(())
    }

    async fn update_time_series_cache(&self, data: &SensorData) -> Result<()> {
        let key = format!("{}:{}", data.device_id, data.sensor_type);
        let data_point = DataPoint {
            timestamp: data.timestamp,
            value: data.value,
            quality: data.quality,
        };
        
        // 更新内存缓存
        let mut cache_entry = self.time_series_cache.entry(key.clone()).or_insert_with(Vec::new);
        cache_entry.push(data_point.clone());
        
        // 限制缓存大小
        if cache_entry.len() > 1000 {
            cache_entry.drain(0..100); // 移除最旧的100个数据点
        }
        
        // 更新Redis缓存
        let mut conn = self.redis_client.get_async_connection().await?;
        let serialized = serde_json::to_string(&data_point)?;
        redis::cmd("LPUSH")
            .arg(&key)
            .arg(&serialized)
            .query_async::<_, ()>(&mut conn)
            .await?;
        
        // 限制Redis列表长度
        redis::cmd("LTRIM")
            .arg(&key)
            .arg(0)
            .arg(999)
            .query_async::<_, ()>(&mut conn)
            .await?;
        
        Ok(())
    }

    async fn push_real_time_data(&self, data: &SensorData) -> Result<()> {
        // 实际实现中会通过WebSocket或SSE推送实时数据
        // 这里只是模拟
        debug!("Pushing real-time data: {:?}", data.id);
        Ok(())
    }

    async fn get_historical_average(&self, device_id: &Uuid, sensor_type: &str) -> Result<Option<f64>> {
        let result = sqlx::query!(
            "SELECT AVG(value) as avg_value FROM sensor_data WHERE device_id = $1 AND sensor_type = $2 AND timestamp > NOW() - INTERVAL '24 hours'",
            device_id,
            sensor_type
        )
        .fetch_optional(&self.db_pool)
        .await?;
        
        Ok(result.and_then(|row| row.avg_value))
    }

    pub async fn get_time_series(&self, device_id: Uuid, sensor_type: &str, duration: Duration) -> Result<TimeSeries> {
        let start_time = Utc::now() - chrono::Duration::from_std(duration)?;
        
        let rows = sqlx::query!(
            "SELECT value, quality, timestamp FROM sensor_data WHERE device_id = $1 AND sensor_type = $2 AND timestamp >= $3 ORDER BY timestamp",
            device_id,
            sensor_type,
            start_time
        )
        .fetch_all(&self.db_pool)
        .await?;
        
        let data_points: Vec<DataPoint> = rows
            .into_iter()
            .map(|row| DataPoint {
                timestamp: row.timestamp,
                value: row.value,
                quality: row.quality,
            })
            .collect();
        
        Ok(TimeSeries {
            device_id,
            sensor_type: sensor_type.to_string(),
            data_points,
            aggregation: None,
            interval: None,
        })
    }
}

// 规则引擎
pub struct RuleEngine {
    rules: Arc<RwLock<HashMap<Uuid, Rule>>>,
    db_pool: PgPool,
    alert_sender: broadcast::Sender<Alert>,
}

impl RuleEngine {
    pub fn new(db_pool: PgPool) -> Self {
        let (alert_sender, _) = broadcast::channel(1000);
        
        Self {
            rules: Arc::new(RwLock::new(HashMap::new())),
            db_pool,
            alert_sender,
        }
    }

    pub async fn add_rule(&self, mut rule: Rule) -> Result<()> {
        rule.created_at = Utc::now();
        rule.updated_at = Utc::now();
        
        // 保存到数据库
        sqlx::query!(
            r#"
            INSERT INTO rules (id, name, description, rule_type, conditions, actions, enabled, priority, created_at, updated_at)
            VALUES ($1, $2, $3, $4, $5, $6, $7, $8, $9, $10)
            "#,
            rule.id,
            rule.name,
            rule.description,
            serde_json::to_string(&rule.rule_type)?,
            serde_json::to_string(&rule.conditions)?,
            serde_json::to_string(&rule.actions)?,
            rule.enabled,
            rule.priority as i16,
            rule.created_at,
            rule.updated_at
        )
        .execute(&self.db_pool)
        .await?;
        
        // 添加到内存
        let mut rules = self.rules.write().await;
        rules.insert(rule.id, rule.clone());
        
        info!("Rule added: {} ({})", rule.name, rule.id);
        Ok(())
    }

    pub async fn evaluate_rules(&self, data: &SensorData) -> Result<()> {
        let rules = self.rules.read().await;
        
        for rule in rules.values() {
            if !rule.enabled {
                continue;
            }
            
            if self.evaluate_rule(rule, data).await? {
                self.execute_rule_actions(rule, data).await?;
            }
        }
        
        Ok(())
    }

    async fn evaluate_rule(&self, rule: &Rule, data: &SensorData) -> Result<bool> {
        let mut result = true;
        let mut last_logical_op = None;
        
        for condition in &rule.conditions {
            let condition_result = self.evaluate_condition(condition, data).await?;
            
            match last_logical_op.as_deref() {
                Some("AND") => result = result && condition_result,
                Some("OR") => result = result || condition_result,
                None => result = condition_result,
                _ => return Err(anyhow::anyhow!("Unknown logical operator")),
            }
            
            last_logical_op = condition.logical_op.clone();
        }
        
        Ok(result)
    }

    async fn evaluate_condition(&self, condition: &RuleCondition, data: &SensorData) -> Result<bool> {
        let field_value = match condition.field.as_str() {
            "value" => data.value.to_string(),
            "quality" => data.quality.to_string(),
            "sensor_type" => data.sensor_type.clone(),
            "device_id" => data.device_id.to_string(),
            _ => return Ok(false),
        };
        
        match condition.operator.as_str() {
            ">" => {
                let threshold: f64 = condition.value.parse()?;
                let value: f64 = field_value.parse()?;
                Ok(value > threshold)
            }
            "<" => {
                let threshold: f64 = condition.value.parse()?;
                let value: f64 = field_value.parse()?;
                Ok(value < threshold)
            }
            "==" => Ok(field_value == condition.value),
            "!=" => Ok(field_value != condition.value),
            "contains" => Ok(field_value.contains(&condition.value)),
            _ => Err(anyhow::anyhow!("Unknown operator: {}", condition.operator)),
        }
    }

    async fn execute_rule_actions(&self, rule: &Rule, data: &SensorData) -> Result<()> {
        for action in &rule.actions {
            match action.action_type.as_str() {
                "alert" => {
                    let alert = Alert {
                        id: Uuid::new_v4(),
                        rule_id: rule.id,
                        device_id: Some(data.device_id),
                        level: match rule.priority {
                            1..=3 => AlertLevel::Info,
                            4..=6 => AlertLevel::Warning,
                            7..=8 => AlertLevel::Error,
                            9..=10 => AlertLevel::Critical,
                            _ => AlertLevel::Info,
                        },
                        title: action.parameters.get("title")
                            .unwrap_or(&format!("Rule triggered: {}", rule.name))
                            .clone(),
                        message: action.parameters.get("message")
                            .unwrap_or(&format!("Rule {} was triggered by device {}", rule.name, data.device_id))
                            .clone(),
                        data: HashMap::new(),
                        acknowledged: false,
                        resolved: false,
                        created_at: Utc::now(),
                        resolved_at: None,
                    };
                    
                    self.create_alert(alert).await?;
                }
                "webhook" => {
                    self.send_webhook(action, data).await?;
                }
                "control" => {
                    self.send_control_command(action, data).await?;
                }
                _ => {
                    warn!("Unknown action type: {}", action.action_type);
                }
            }
        }
        
        Ok(())
    }

    async fn create_alert(&self, alert: Alert) -> Result<()> {
        // 保存到数据库
        sqlx::query!(
            r#"
            INSERT INTO alerts (id, rule_id, device_id, level, title, message, data, acknowledged, resolved, created_at)
            VALUES ($1, $2, $3, $4, $5, $6, $7, $8, $9, $10)
            "#,
            alert.id,
            alert.rule_id,
            alert.device_id,
            serde_json::to_string(&alert.level)?,
            alert.title,
            alert.message,
            serde_json::to_string(&alert.data)?,
            alert.acknowledged,
            alert.resolved,
            alert.created_at
        )
        .execute(&self.db_pool)
        .await?;
        
        // 发送告警事件
        let _ = self.alert_sender.send(alert.clone());
        
        info!("Alert created: {} ({})", alert.title, alert.id);
        Ok(())
    }

    async fn send_webhook(&self, action: &RuleAction, data: &SensorData) -> Result<()> {
        let client = reqwest::Client::new();
        let payload = serde_json::json!({
            "action": "webhook",
            "target": action.target,
            "data": data,
            "parameters": action.parameters
        });
        
        let response = client
            .post(&action.target)
            .json(&payload)
            .send()
            .await?;
        
        if response.status().is_success() {
            debug!("Webhook sent successfully to {}", action.target);
        } else {
            warn!("Webhook failed: {} - {}", response.status(), action.target);
        }
        
        Ok(())
    }

    async fn send_control_command(&self, action: &RuleAction, _data: &SensorData) -> Result<()> {
        // 实际实现中会发送控制命令到设备
        debug!("Control command sent to {}: {:?}", action.target, action.parameters);
        Ok(())
    }

    pub fn subscribe_alerts(&self) -> broadcast::Receiver<Alert> {
        self.alert_sender.subscribe()
    }
}

// IoT平台核心
pub struct IoTPlatform {
    device_manager: Arc<DeviceManager>,
    data_processor: Arc<DataProcessor>,
    rule_engine: Arc<RuleEngine>,
    db_pool: PgPool,
}

impl IoTPlatform {
    pub async fn new(database_url: &str, redis_url: &str) -> Result<Self> {
        // 初始化数据库连接池
        let db_pool = PgPool::connect(database_url).await?;
        
        // 运行数据库迁移
        sqlx::migrate!("./migrations").run(&db_pool).await?;
        
        // 初始化组件
        let device_manager = Arc::new(DeviceManager::new(db_pool.clone()));
        let data_processor = Arc::new(DataProcessor::new(db_pool.clone(), redis_url)?);
        let rule_engine = Arc::new(RuleEngine::new(db_pool.clone()));
        
        Ok(Self {
            device_manager,
            data_processor,
            rule_engine,
            db_pool,
        })
    }

    pub async fn start(&self) -> Result<()> {
        info!("Starting IoT Platform...");
        
        // 启动事件处理器
        self.start_event_processors().await?;
        
        // 启动Web API服务器
        self.start_web_server().await?;
        
        info!("IoT Platform started successfully");
        Ok(())
    }

    async fn start_event_processors(&self) -> Result<()> {
        // 设备事件处理器
        let device_manager = self.device_manager.clone();
        let data_processor = self.data_processor.clone();
        let rule_engine = self.rule_engine.clone();
        
        let mut device_events = device_manager.subscribe_events();
        tokio::spawn(async move {
            while let Ok(event) = device_events.recv().await {
                match event {
                    DeviceEvent::DataReceived(device_id, data) => {
                        if let Err(e) = data_processor.process_sensor_data(data.clone()).await {
                            error!("Failed to process sensor data: {}", e);
                        }
                        
                        if let Err(e) = rule_engine.evaluate_rules(&data).await {
                            error!("Failed to evaluate rules: {}", e);
                        }
                    }
                    DeviceEvent::Connected(device_id) => {
                        info!("Device connected: {}", device_id);
                    }
                    DeviceEvent::Disconnected(device_id) => {
                        warn!("Device disconnected: {}", device_id);
                    }
                    DeviceEvent::StatusChanged(device_id, status) => {
                        info!("Device status changed: {} -> {:?}", device_id, status);
                    }
                }
            }
        });
        
        // 告警事件处理器
        let mut alert_events = self.rule_engine.subscribe_alerts();
        tokio::spawn(async move {
            while let Ok(alert) = alert_events.recv().await {
                info!("Alert generated: {} - {}", alert.level, alert.title);
                // 这里可以添加告警通知逻辑，如发送邮件、短信等
            }
        });
        
        Ok(())
    }

    async fn start_web_server(&self) -> Result<()> {
        let app_state = AppState {
            platform: Arc::new(self.clone()),
        };
        
        let app = Router::new()
            .route("/api/devices", get(list_devices).post(register_device))
            .route("/api/devices/:id", get(get_device).put(update_device).delete(delete_device))
            .route("/api/devices/:id/data", get(get_device_data))
            .route("/api/data", post(ingest_data))
            .route("/api/rules", get(list_rules).post(create_rule))
            .route("/api/rules/:id", get(get_rule).put(update_rule).delete(delete_rule))
            .route("/api/alerts", get(list_alerts))
            .route("/api/alerts/:id/acknowledge", post(acknowledge_alert))
            .route("/api/health", get(health_check))
            .layer(CorsLayer::permissive())
            .with_state(app_state);
        
        let listener = tokio::net::TcpListener::bind("0.0.0.0:8080").await?;
        info!("Web server listening on http://0.0.0.0:8080");
        
        axum::serve(listener, app).await?;
        
        Ok(())
    }
}

impl Clone for IoTPlatform {
    fn clone(&self) -> Self {
        Self {
            device_manager: self.device_manager.clone(),
            data_processor: self.data_processor.clone(),
            rule_engine: self.rule_engine.clone(),
            db_pool: self.db_pool.clone(),
        }
    }
}

// Web API状态
#[derive(Clone)]
struct AppState {
    platform: Arc<IoTPlatform>,
}

// API处理函数
async fn list_devices(State(state): State<AppState>) -> Result<Json<Vec<Device>>, StatusCode> {
    let devices = state.platform.device_manager.list_devices();
    Ok(Json(devices))
}

async fn register_device(
    State(state): State<AppState>,
    Json(device): Json<Device>,
) -> Result<Json<Device>, StatusCode> {
    state.platform.device_manager.register_device(device.clone()).await
        .map_err(|_| StatusCode::INTERNAL_SERVER_ERROR)?;
    Ok(Json(device))
}

async fn get_device(
    State(state): State<AppState>,
    Path(id): Path<Uuid>,
) -> Result<Json<Device>, StatusCode> {
    match state.platform.device_manager.get_device(&id) {
        Some(device) => Ok(Json(device)),
        None => Err(StatusCode::NOT_FOUND),
    }
}

async fn update_device(
    State(state): State<AppState>,
    Path(id): Path<Uuid>,
    Json(device): Json<Device>,
) -> Result<Json<Device>, StatusCode> {
    // 实际实现中会更新设备信息
    Ok(Json(device))
}

async fn delete_device(
    State(state): State<AppState>,
    Path(id): Path<Uuid>,
) -> Result<StatusCode, StatusCode> {
    // 实际实现中会删除设备
    Ok(StatusCode::NO_CONTENT)
}

#[derive(Deserialize)]
struct DataQuery {
    duration: Option<u64>, // 秒
    sensor_type: Option<String>,
}

async fn get_device_data(
    State(state): State<AppState>,
    Path(id): Path<Uuid>,
    Query(query): Query<DataQuery>,
) -> Result<Json<TimeSeries>, StatusCode> {
    let duration = Duration::from_secs(query.duration.unwrap_or(3600)); // 默认1小时
    let sensor_type = query.sensor_type.unwrap_or_else(|| "temperature".to_string());
    
    match state.platform.data_processor.get_time_series(id, &sensor_type, duration).await {
        Ok(time_series) => Ok(Json(time_series)),
        Err(_) => Err(StatusCode::INTERNAL_SERVER_ERROR),
    }
}

async fn ingest_data(
    State(state): State<AppState>,
    Json(data): Json<SensorData>,
) -> Result<StatusCode, StatusCode> {
    state.platform.data_processor.process_sensor_data(data).await
        .map_err(|_| StatusCode::INTERNAL_SERVER_ERROR)?;
    Ok(StatusCode::ACCEPTED)
}

async fn list_rules(State(state): State<AppState>) -> Result<Json<Vec<Rule>>, StatusCode> {
    // 实际实现中会从数据库获取规则列表
    Ok(Json(vec![]))
}

async fn create_rule(
    State(state): State<AppState>,
    Json(rule): Json<Rule>,
) -> Result<Json<Rule>, StatusCode> {
    state.platform.rule_engine.add_rule(rule.clone()).await
        .map_err(|_| StatusCode::INTERNAL_SERVER_ERROR)?;
    Ok(Json(rule))
}

async fn get_rule(
    State(state): State<AppState>,
    Path(id): Path<Uuid>,
) -> Result<Json<Rule>, StatusCode> {
    // 实际实现中会获取特定规则
    Err(StatusCode::NOT_FOUND)
}

async fn update_rule(
    State(state): State<AppState>,
    Path(id): Path<Uuid>,
    Json(rule): Json<Rule>,
) -> Result<Json<Rule>, StatusCode> {
    // 实际实现中会更新规则
    Ok(Json(rule))
}

async fn delete_rule(
    State(state): State<AppState>,
    Path(id): Path<Uuid>,
) -> Result<StatusCode, StatusCode> {
    // 实际实现中会删除规则
    Ok(StatusCode::NO_CONTENT)
}

async fn list_alerts(State(state): State<AppState>) -> Result<Json<Vec<Alert>>, StatusCode> {
    // 实际实现中会从数据库获取告警列表
    Ok(Json(vec![]))
}

async fn acknowledge_alert(
    State(state): State<AppState>,
    Path(id): Path<Uuid>,
) -> Result<StatusCode, StatusCode> {
    // 实际实现中会确认告警
    Ok(StatusCode::OK)
}

async fn health_check() -> Json<serde_json::Value> {
    Json(serde_json::json!({
        "status": "healthy",
        "timestamp": Utc::now(),
        "version": env!("CARGO_PKG_VERSION")
    }))
}

#[tokio::main]
async fn main() -> Result<()> {
    // 初始化日志
    tracing_subscriber::fmt()
        .with_env_filter("iot_platform=debug,info")
        .init();
    
    info!("Starting IoT Platform System");
    
    // 从环境变量获取配置
    let database_url = std::env::var("DATABASE_URL")
        .unwrap_or_else(|_| "postgresql://localhost/iot_platform".to_string());
    let redis_url = std::env::var("REDIS_URL")
        .unwrap_or_else(|_| "redis://localhost:6379".to_string());
    
    // 创建IoT平台
    let platform = IoTPlatform::new(&database_url, &redis_url).await
        .context("Failed to create IoT platform")?;
    
    // 启动平台
    platform.start().await
        .context("Failed to start IoT platform")?;
    
    Ok(())
}