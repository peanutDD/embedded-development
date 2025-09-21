//! Web HMI (人机界面) 示例
//! 
//! 这个示例展示了基于Web的工业人机界面系统：
//! - HTTP服务器
//! - 实时数据展示
//! - 用户交互控制
//! - 报警管理
//! - 历史数据查询

use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

/// 数据点结构
#[cfg_attr(feature = "rest-api", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone)]
struct DataPoint {
    tag_name: String,
    value: f64,
    timestamp: u64,
    quality: String,
    unit: String,
}

/// 报警信息
#[cfg_attr(feature = "rest-api", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone)]
struct Alarm {
    id: u32,
    tag_name: String,
    message: String,
    severity: AlarmSeverity,
    timestamp: u64,
    acknowledged: bool,
}

/// 报警严重程度
#[cfg_attr(feature = "rest-api", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone)]
enum AlarmSeverity {
    Info,
    Warning,
    Error,
    Critical,
}

/// 用户命令
#[cfg_attr(feature = "rest-api", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone)]
struct UserCommand {
    command_type: String,
    target: String,
    value: Option<f64>,
    user: String,
    timestamp: u64,
}

/// 系统状态
#[cfg_attr(feature = "rest-api", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone)]
struct SystemStatus {
    running: bool,
    connected_users: u32,
    active_alarms: u32,
    data_points_count: u32,
    uptime: u64,
}

/// Web HMI服务器
struct WebHmiServer {
    data_points: Arc<Mutex<HashMap<String, DataPoint>>>,
    alarms: Arc<Mutex<Vec<Alarm>>>,
    commands: Arc<Mutex<Vec<UserCommand>>>,
    connected_users: Arc<Mutex<u32>>,
    is_running: Arc<Mutex<bool>>,
    alarm_counter: Arc<Mutex<u32>>,
}

impl WebHmiServer {
    fn new() -> Self {
        Self {
            data_points: Arc::new(Mutex::new(HashMap::new())),
            alarms: Arc::new(Mutex::new(Vec::new())),
            commands: Arc::new(Mutex::new(Vec::new())),
            connected_users: Arc::new(Mutex::new(0)),
            is_running: Arc::new(Mutex::new(false)),
            alarm_counter: Arc::new(Mutex::new(0)),
        }
    }
    
    /// 启动Web服务器
    fn start(&self, port: u16) {
        {
            let mut running = self.is_running.lock().unwrap();
            *running = true;
        }
        
        println!("Web HMI服务器启动在端口 {}", port);
        
        // 启动数据模拟线程
        self.start_data_simulator();
        
        // 启动报警监控线程
        self.start_alarm_monitor();
        
        // 模拟HTTP服务器
        self.start_http_server(port);
    }
    
    /// 停止服务器
    fn stop(&self) {
        let mut running = self.is_running.lock().unwrap();
        *running = false;
        println!("Web HMI服务器已停止");
    }
    
    /// 启动数据模拟线程
    fn start_data_simulator(&self) {
        let data_points = Arc::clone(&self.data_points);
        let is_running = Arc::clone(&self.is_running);
        
        thread::spawn(move || {
            let mut counter = 0;
            
            while *is_running.lock().unwrap() {
                let timestamp = SystemTime::now()
                    .duration_since(UNIX_EPOCH)
                    .unwrap()
                    .as_secs();
                
                {
                    let mut data = data_points.lock().unwrap();
                    
                    // 模拟温度数据
                    data.insert("Temperature_01".to_string(), DataPoint {
                        tag_name: "Temperature_01".to_string(),
                        value: 25.0 + 10.0 * (counter as f64 * 0.1).sin(),
                        timestamp,
                        quality: "Good".to_string(),
                        unit: "°C".to_string(),
                    });
                    
                    // 模拟压力数据
                    data.insert("Pressure_01".to_string(), DataPoint {
                        tag_name: "Pressure_01".to_string(),
                        value: 2.0 + 1.0 * (counter as f64 * 0.05).cos(),
                        timestamp,
                        quality: "Good".to_string(),
                        unit: "bar".to_string(),
                    });
                    
                    // 模拟流量数据
                    data.insert("Flow_01".to_string(), DataPoint {
                        tag_name: "Flow_01".to_string(),
                        value: 100.0 + 20.0 * (counter as f64 * 0.02).sin(),
                        timestamp,
                        quality: "Good".to_string(),
                        unit: "L/min".to_string(),
                    });
                    
                    // 模拟电机转速
                    data.insert("Motor_Speed_01".to_string(), DataPoint {
                        tag_name: "Motor_Speed_01".to_string(),
                        value: 1500.0 + 100.0 * (counter as f64 * 0.03).cos(),
                        timestamp,
                        quality: "Good".to_string(),
                        unit: "RPM".to_string(),
                    });
                }
                
                counter += 1;
                thread::sleep(Duration::from_secs(1));
            }
        });
    }
    
    /// 启动报警监控线程
    fn start_alarm_monitor(&self) {
        let data_points = Arc::clone(&self.data_points);
        let alarms = Arc::clone(&self.alarms);
        let alarm_counter = Arc::clone(&self.alarm_counter);
        let is_running = Arc::clone(&self.is_running);
        
        thread::spawn(move || {
            while *is_running.lock().unwrap() {
                {
                    let data = data_points.lock().unwrap();
                    let mut alarms = alarms.lock().unwrap();
                    let mut counter = alarm_counter.lock().unwrap();
                    
                    // 检查温度报警
                    if let Some(temp_point) = data.get("Temperature_01") {
                        if temp_point.value > 30.0 {
                            *counter += 1;
                            alarms.push(Alarm {
                                id: *counter,
                                tag_name: "Temperature_01".to_string(),
                                message: format!("温度过高: {:.1}°C", temp_point.value),
                                severity: AlarmSeverity::Warning,
                                timestamp: temp_point.timestamp,
                                acknowledged: false,
                            });
                        }
                    }
                    
                    // 检查压力报警
                    if let Some(pressure_point) = data.get("Pressure_01") {
                        if pressure_point.value > 2.8 {
                            *counter += 1;
                            alarms.push(Alarm {
                                id: *counter,
                                tag_name: "Pressure_01".to_string(),
                                message: format!("压力过高: {:.1}bar", pressure_point.value),
                                severity: AlarmSeverity::Error,
                                timestamp: pressure_point.timestamp,
                                acknowledged: false,
                            });
                        }
                    }
                    
                    // 限制报警数量
                    if alarms.len() > 100 {
                        alarms.drain(0..50);
                    }
                }
                
                thread::sleep(Duration::from_secs(2));
            }
        });
    }
    
    /// 启动HTTP服务器（模拟）
    fn start_http_server(&self, port: u16) {
        let data_points = Arc::clone(&self.data_points);
        let alarms = Arc::clone(&self.alarms);
        let commands = Arc::clone(&self.commands);
        let connected_users = Arc::clone(&self.connected_users);
        let is_running = Arc::clone(&self.is_running);
        
        thread::spawn(move || {
            println!("HTTP服务器监听端口 {}", port);
            
            while *is_running.lock().unwrap() {
                // 模拟处理HTTP请求
                Self::simulate_http_requests(
                    &data_points,
                    &alarms,
                    &commands,
                    &connected_users,
                );
                
                thread::sleep(Duration::from_millis(100));
            }
        });
    }
    
    /// 模拟HTTP请求处理
    fn simulate_http_requests(
        _data_points: &Arc<Mutex<HashMap<String, DataPoint>>>,
        _alarms: &Arc<Mutex<Vec<Alarm>>>,
        commands: &Arc<Mutex<Vec<UserCommand>>>,
        connected_users: &Arc<Mutex<u32>>,
    ) {
        // 模拟用户连接
        {
            let mut users = connected_users.lock().unwrap();
            *users = 3; // 假设有3个用户连接
        }
        
        // 模拟用户命令
        {
            let mut commands = commands.lock().unwrap();
            
            // 每隔一段时间添加一个模拟命令
            use std::collections::hash_map::DefaultHasher;
            use std::hash::{Hash, Hasher};
            
            let mut hasher = DefaultHasher::new();
            SystemTime::now().hash(&mut hasher);
            let random = hasher.finish();
            
            if random % 100 == 0 {
                commands.push(UserCommand {
                    command_type: "SET_SETPOINT".to_string(),
                    target: "Temperature_01".to_string(),
                    value: Some(28.0),
                    user: "operator1".to_string(),
                    timestamp: SystemTime::now()
                        .duration_since(UNIX_EPOCH)
                        .unwrap()
                        .as_secs(),
                });
            }
        }
    }
    
    /// 获取实时数据API
    fn get_realtime_data(&self) -> Vec<DataPoint> {
        let data = self.data_points.lock().unwrap();
        data.values().cloned().collect()
    }
    
    /// 获取报警列表API
    fn get_alarms(&self) -> Vec<Alarm> {
        let alarms = self.alarms.lock().unwrap();
        alarms.clone()
    }
    
    /// 确认报警API
    fn acknowledge_alarm(&self, alarm_id: u32, user: &str) -> bool {
        let mut alarms = self.alarms.lock().unwrap();
        
        if let Some(alarm) = alarms.iter_mut().find(|a| a.id == alarm_id) {
            alarm.acknowledged = true;
            println!("用户 {} 确认了报警 {}", user, alarm_id);
            true
        } else {
            false
        }
    }
    
    /// 发送用户命令API
    fn send_command(&self, command: UserCommand) -> bool {
        let mut commands = self.commands.lock().unwrap();
        
        println!("收到用户命令: {:?}", command);
        commands.push(command);
        
        // 限制命令历史数量
        if commands.len() > 1000 {
            commands.drain(0..500);
        }
        
        true
    }
    
    /// 获取系统状态API
    fn get_system_status(&self) -> SystemStatus {
        let data_count = self.data_points.lock().unwrap().len() as u32;
        let active_alarms = self.alarms.lock().unwrap()
            .iter()
            .filter(|a| !a.acknowledged)
            .count() as u32;
        let users = *self.connected_users.lock().unwrap();
        let running = *self.is_running.lock().unwrap();
        
        SystemStatus {
            running,
            connected_users: users,
            active_alarms,
            data_points_count: data_count,
            uptime: 3600, // 模拟运行时间
        }
    }
    
    /// 生成HTML仪表板（简化版）
    fn generate_dashboard_html(&self) -> String {
        let data = self.get_realtime_data();
        let alarms = self.get_alarms();
        let status = self.get_system_status();
        
        let mut html = format!(r#"
<!DOCTYPE html>
<html>
<head>
    <title>工业HMI仪表板</title>
    <meta charset="UTF-8">
    <style>
        body {{ font-family: Arial, sans-serif; margin: 20px; }}
        .dashboard {{ display: grid; grid-template-columns: 1fr 1fr; gap: 20px; }}
        .panel {{ border: 1px solid #ccc; padding: 15px; border-radius: 5px; }}
        .data-point {{ margin: 10px 0; padding: 10px; background: #f5f5f5; }}
        .alarm {{ margin: 5px 0; padding: 8px; border-radius: 3px; }}
        .alarm.warning {{ background: #fff3cd; }}
        .alarm.error {{ background: #f8d7da; }}
        .alarm.critical {{ background: #d1ecf1; }}
        .status {{ background: #d4edda; }}
    </style>
</head>
<body>
    <h1>工业HMI仪表板</h1>
    
    <div class="panel status">
        <h3>系统状态</h3>
        <p>运行状态: {}</p>
        <p>连接用户: {}</p>
        <p>活跃报警: {}</p>
        <p>数据点数: {}</p>
    </div>
    
    <div class="dashboard">
        <div class="panel">
            <h3>实时数据</h3>
"#,
            if status.running { "运行中" } else { "停止" },
            status.connected_users,
            status.active_alarms,
            status.data_points_count
        );
        
        // 添加数据点
        for point in data {
            html.push_str(&format!(
                r#"            <div class="data-point">
                <strong>{}:</strong> {:.2} {}<br>
                <small>时间: {}</small>
            </div>
"#,
                point.tag_name,
                point.value,
                point.unit,
                point.timestamp
            ));
        }
        
        html.push_str(r#"
        </div>
        
        <div class="panel">
            <h3>报警信息</h3>
"#);
        
        // 添加报警
        let recent_alarms: Vec<_> = alarms.iter().rev().take(10).collect();
        for alarm in recent_alarms {
            let class = match alarm.severity {
                AlarmSeverity::Warning => "warning",
                AlarmSeverity::Error => "error",
                AlarmSeverity::Critical => "critical",
                _ => "info",
            };
            
            html.push_str(&format!(
                r#"            <div class="alarm {}">
                <strong>{}</strong><br>
                {}<br>
                <small>时间: {} | 状态: {}</small>
            </div>
"#,
                class,
                alarm.tag_name,
                alarm.message,
                alarm.timestamp,
                if alarm.acknowledged { "已确认" } else { "未确认" }
            ));
        }
        
        html.push_str(r#"
        </div>
    </div>
    
    <script>
        // 自动刷新页面
        setTimeout(function() {
            location.reload();
        }, 5000);
    </script>
</body>
</html>
"#);
        
        html
    }
}

fn main() {
    println!("Web HMI (人机界面) 示例");
    
    let hmi_server = WebHmiServer::new();
    
    // 启动Web服务器
    hmi_server.start(8080);
    
    println!("\nWeb HMI服务器运行中...");
    println!("访问 http://localhost:8080 查看仪表板");
    
    // 运行一段时间并展示功能
    for i in 0..30 {
        thread::sleep(Duration::from_secs(1));
        
        if i % 5 == 0 {
            // 显示实时数据
            let data = hmi_server.get_realtime_data();
            println!("\n=== 实时数据 ({}s) ===", i);
            for point in data.iter().take(2) {
                println!("{}: {:.2} {}", point.tag_name, point.value, point.unit);
            }
        }
        
        if i % 10 == 0 {
            // 显示报警状态
            let alarms = hmi_server.get_alarms();
            let active_alarms: Vec<_> = alarms.iter()
                .filter(|a| !a.acknowledged)
                .collect();
            
            println!("\n活跃报警数: {}", active_alarms.len());
            
            // 模拟确认一些报警
            if let Some(alarm) = active_alarms.first() {
                hmi_server.acknowledge_alarm(alarm.id, "operator1");
            }
        }
        
        if i == 15 {
            // 模拟发送用户命令
            let command = UserCommand {
                command_type: "START_MOTOR".to_string(),
                target: "Motor_01".to_string(),
                value: None,
                user: "operator2".to_string(),
                timestamp: SystemTime::now()
                    .duration_since(UNIX_EPOCH)
                    .unwrap()
                    .as_secs(),
            };
            
            hmi_server.send_command(command);
        }
        
        if i == 20 {
            // 生成并显示HTML仪表板示例
            let html = hmi_server.generate_dashboard_html();
            println!("\n=== HTML仪表板示例 ===");
            println!("HTML长度: {} 字符", html.len());
            println!("包含实时数据和报警信息的完整仪表板");
        }
    }
    
    // 显示最终系统状态
    let final_status = hmi_server.get_system_status();
    println!("\n=== 最终系统状态 ===");
    println!("运行状态: {}", final_status.running);
    println!("连接用户: {}", final_status.connected_users);
    println!("活跃报警: {}", final_status.active_alarms);
    println!("数据点数: {}", final_status.data_points_count);
    
    // 停止服务器
    hmi_server.stop();
    
    println!("\nWeb HMI示例完成");
}

// 注意：这个示例使用了serde进行序列化，实际项目中需要添加以下依赖：
// [dependencies]
// serde = { version = "1.0", features = ["derive"] }
// serde_json = "1.0"

// 为了编译通过，这里提供简化的serde模块
// 注意：在实际项目中，需要在Cargo.toml中添加serde依赖：
// serde = { version = "1.0", features = ["derive"] }