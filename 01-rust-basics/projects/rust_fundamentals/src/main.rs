// Rust基础综合项目 - 展示所有核心概念的实际应用
// 这个项目整合了所有Rust基础知识点，创建一个功能完整的应用程序

use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};
use std::fs;
use std::io::{self, Write};

use serde::{Deserialize, Serialize};
use chrono::{DateTime, Utc};
use rand::Rng;
use log::{info, warn, error, debug};

// 1. 自定义错误类型
#[derive(Debug, thiserror::Error)]
pub enum AppError {
    #[error("IO错误: {0}")]
    Io(#[from] std::io::Error),
    
    #[error("序列化错误: {0}")]
    Serialization(#[from] serde_json::Error),
    
    #[error("配置错误: {message}")]
    Config { message: String },
    
    #[error("业务逻辑错误: {0}")]
    Business(String),
    
    #[error("网络错误: {0}")]
    Network(String),
}

type Result<T> = std::result::Result<T, AppError>;

// 2. 核心数据结构
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct User {
    pub id: u64,
    pub name: String,
    pub email: String,
    pub created_at: DateTime<Utc>,
    pub is_active: bool,
    pub metadata: HashMap<String, String>,
}

impl User {
    pub fn new(id: u64, name: String, email: String) -> Self {
        Self {
            id,
            name,
            email,
            created_at: Utc::now(),
            is_active: true,
            metadata: HashMap::new(),
        }
    }
    
    pub fn add_metadata(&mut self, key: String, value: String) {
        self.metadata.insert(key, value);
    }
    
    pub fn deactivate(&mut self) {
        self.is_active = false;
    }
    
    pub fn validate(&self) -> Result<()> {
        if self.name.is_empty() {
            return Err(AppError::Business("用户名不能为空".to_string()));
        }
        
        if !self.email.contains('@') {
            return Err(AppError::Business("邮箱格式无效".to_string()));
        }
        
        Ok(())
    }
}

// 3. 任务系统
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Task {
    pub id: u64,
    pub title: String,
    pub description: String,
    pub status: TaskStatus,
    pub assigned_to: Option<u64>,
    pub created_at: DateTime<Utc>,
    pub updated_at: DateTime<Utc>,
    pub priority: Priority,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum TaskStatus {
    Pending,
    InProgress,
    Completed,
    Cancelled,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialOrd, PartialEq)]
pub enum Priority {
    Low,
    Medium,
    High,
    Critical,
}

impl Task {
    pub fn new(id: u64, title: String, description: String) -> Self {
        let now = Utc::now();
        Self {
            id,
            title,
            description,
            status: TaskStatus::Pending,
            assigned_to: None,
            created_at: now,
            updated_at: now,
            priority: Priority::Medium,
        }
    }
    
    pub fn assign_to(&mut self, user_id: u64) {
        self.assigned_to = Some(user_id);
        self.updated_at = Utc::now();
    }
    
    pub fn update_status(&mut self, status: TaskStatus) {
        self.status = status;
        self.updated_at = Utc::now();
    }
    
    pub fn set_priority(&mut self, priority: Priority) {
        self.priority = priority;
        self.updated_at = Utc::now();
    }
}

// 4. 应用程序状态管理
#[derive(Debug)]
pub struct AppState {
    users: Arc<Mutex<HashMap<u64, User>>>,
    tasks: Arc<Mutex<HashMap<u64, Task>>>,
    next_user_id: Arc<Mutex<u64>>,
    next_task_id: Arc<Mutex<u64>>,
    stats: Arc<Mutex<AppStats>>,
}

#[derive(Debug, Default)]
pub struct AppStats {
    pub total_users: u64,
    pub total_tasks: u64,
    pub completed_tasks: u64,
    pub active_users: u64,
    pub uptime: Duration,
    pub start_time: Option<Instant>,
}

impl AppState {
    pub fn new() -> Self {
        Self {
            users: Arc::new(Mutex::new(HashMap::new())),
            tasks: Arc::new(Mutex::new(HashMap::new())),
            next_user_id: Arc::new(Mutex::new(1)),
            next_task_id: Arc::new(Mutex::new(1)),
            stats: Arc::new(Mutex::new(AppStats {
                start_time: Some(Instant::now()),
                ..Default::default()
            })),
        }
    }
    
    // 用户管理
    pub fn create_user(&self, name: String, email: String) -> Result<u64> {
        let mut users = self.users.lock().unwrap();
        let mut next_id = self.next_user_id.lock().unwrap();
        let mut stats = self.stats.lock().unwrap();
        
        let user = User::new(*next_id, name, email);
        user.validate()?;
        
        users.insert(*next_id, user);
        stats.total_users += 1;
        stats.active_users += 1;
        
        let id = *next_id;
        *next_id += 1;
        
        info!("创建用户: ID={}", id);
        Ok(id)
    }
    
    pub fn get_user(&self, id: u64) -> Option<User> {
        let users = self.users.lock().unwrap();
        users.get(&id).cloned()
    }
    
    pub fn update_user<F>(&self, id: u64, updater: F) -> Result<()>
    where
        F: FnOnce(&mut User) -> Result<()>,
    {
        let mut users = self.users.lock().unwrap();
        match users.get_mut(&id) {
            Some(user) => {
                updater(user)?;
                info!("更新用户: ID={}", id);
                Ok(())
            }
            None => Err(AppError::Business(format!("用户不存在: {}", id))),
        }
    }
    
    pub fn deactivate_user(&self, id: u64) -> Result<()> {
        self.update_user(id, |user| {
            if user.is_active {
                user.deactivate();
                let mut stats = self.stats.lock().unwrap();
                stats.active_users -= 1;
            }
            Ok(())
        })
    }
    
    // 任务管理
    pub fn create_task(&self, title: String, description: String) -> Result<u64> {
        let mut tasks = self.tasks.lock().unwrap();
        let mut next_id = self.next_task_id.lock().unwrap();
        let mut stats = self.stats.lock().unwrap();
        
        let task = Task::new(*next_id, title, description);
        tasks.insert(*next_id, task);
        stats.total_tasks += 1;
        
        let id = *next_id;
        *next_id += 1;
        
        info!("创建任务: ID={}", id);
        Ok(id)
    }
    
    pub fn get_task(&self, id: u64) -> Option<Task> {
        let tasks = self.tasks.lock().unwrap();
        tasks.get(&id).cloned()
    }
    
    pub fn update_task<F>(&self, id: u64, updater: F) -> Result<()>
    where
        F: FnOnce(&mut Task) -> Result<()>,
    {
        let mut tasks = self.tasks.lock().unwrap();
        match tasks.get_mut(&id) {
            Some(task) => {
                let old_status = task.status.clone();
                updater(task)?;
                
                // 更新统计
                if old_status != TaskStatus::Completed && task.status == TaskStatus::Completed {
                    let mut stats = self.stats.lock().unwrap();
                    stats.completed_tasks += 1;
                }
                
                info!("更新任务: ID={}", id);
                Ok(())
            }
            None => Err(AppError::Business(format!("任务不存在: {}", id))),
        }
    }
    
    pub fn assign_task(&self, task_id: u64, user_id: u64) -> Result<()> {
        // 检查用户是否存在
        if self.get_user(user_id).is_none() {
            return Err(AppError::Business(format!("用户不存在: {}", user_id)));
        }
        
        self.update_task(task_id, |task| {
            task.assign_to(user_id);
            Ok(())
        })
    }
    
    pub fn complete_task(&self, task_id: u64) -> Result<()> {
        self.update_task(task_id, |task| {
            task.update_status(TaskStatus::Completed);
            Ok(())
        })
    }
    
    // 查询功能
    pub fn get_user_tasks(&self, user_id: u64) -> Vec<Task> {
        let tasks = self.tasks.lock().unwrap();
        tasks
            .values()
            .filter(|task| task.assigned_to == Some(user_id))
            .cloned()
            .collect()
    }
    
    pub fn get_tasks_by_status(&self, status: TaskStatus) -> Vec<Task> {
        let tasks = self.tasks.lock().unwrap();
        tasks
            .values()
            .filter(|task| task.status == status)
            .cloned()
            .collect()
    }
    
    pub fn get_high_priority_tasks(&self) -> Vec<Task> {
        let tasks = self.tasks.lock().unwrap();
        tasks
            .values()
            .filter(|task| matches!(task.priority, Priority::High | Priority::Critical))
            .cloned()
            .collect()
    }
    
    // 统计功能
    pub fn get_stats(&self) -> AppStats {
        let mut stats = self.stats.lock().unwrap();
        if let Some(start_time) = stats.start_time {
            stats.uptime = start_time.elapsed();
        }
        stats.clone()
    }
    
    // 数据持久化
    pub fn save_to_file(&self, filename: &str) -> Result<()> {
        let users = self.users.lock().unwrap();
        let tasks = self.tasks.lock().unwrap();
        
        let data = serde_json::json!({
            "users": *users,
            "tasks": *tasks,
            "next_user_id": *self.next_user_id.lock().unwrap(),
            "next_task_id": *self.next_task_id.lock().unwrap(),
        });
        
        fs::write(filename, serde_json::to_string_pretty(&data)?)?;
        info!("数据已保存到文件: {}", filename);
        Ok(())
    }
    
    pub fn load_from_file(&self, filename: &str) -> Result<()> {
        let content = fs::read_to_string(filename)?;
        let data: serde_json::Value = serde_json::from_str(&content)?;
        
        if let Some(users_data) = data.get("users") {
            let users: HashMap<u64, User> = serde_json::from_value(users_data.clone())?;
            *self.users.lock().unwrap() = users;
        }
        
        if let Some(tasks_data) = data.get("tasks") {
            let tasks: HashMap<u64, Task> = serde_json::from_value(tasks_data.clone())?;
            *self.tasks.lock().unwrap() = tasks;
        }
        
        if let Some(next_user_id) = data.get("next_user_id").and_then(|v| v.as_u64()) {
            *self.next_user_id.lock().unwrap() = next_user_id;
        }
        
        if let Some(next_task_id) = data.get("next_task_id").and_then(|v| v.as_u64()) {
            *self.next_task_id.lock().unwrap() = next_task_id;
        }
        
        info!("数据已从文件加载: {}", filename);
        Ok(())
    }
}

impl Clone for AppStats {
    fn clone(&self) -> Self {
        Self {
            total_users: self.total_users,
            total_tasks: self.total_tasks,
            completed_tasks: self.completed_tasks,
            active_users: self.active_users,
            uptime: self.uptime,
            start_time: self.start_time,
        }
    }
}

// 5. 并发任务处理器
pub struct TaskProcessor {
    state: Arc<AppState>,
}

impl TaskProcessor {
    pub fn new(state: Arc<AppState>) -> Self {
        Self { state }
    }
    
    pub fn start_background_processing(&self) -> thread::JoinHandle<()> {
        let state = Arc::clone(&self.state);
        
        thread::spawn(move || {
            info!("后台任务处理器启动");
            
            loop {
                // 模拟处理高优先级任务
                let high_priority_tasks = state.get_high_priority_tasks();
                
                for task in high_priority_tasks {
                    if task.status == TaskStatus::Pending {
                        info!("处理高优先级任务: {}", task.title);
                        
                        // 模拟处理时间
                        thread::sleep(Duration::from_millis(100));
                        
                        // 更新任务状态
                        if let Err(e) = state.update_task(task.id, |t| {
                            t.update_status(TaskStatus::InProgress);
                            Ok(())
                        }) {
                            error!("更新任务状态失败: {}", e);
                        }
                    }
                }
                
                thread::sleep(Duration::from_secs(5));
            }
        })
    }
    
    pub fn process_batch_tasks(&self, task_ids: Vec<u64>) -> Vec<Result<()>> {
        let handles: Vec<_> = task_ids
            .into_iter()
            .map(|task_id| {
                let state = Arc::clone(&self.state);
                thread::spawn(move || {
                    info!("处理任务: {}", task_id);
                    thread::sleep(Duration::from_millis(rand::thread_rng().gen_range(50..200)));
                    
                    state.update_task(task_id, |task| {
                        task.update_status(TaskStatus::Completed);
                        Ok(())
                    })
                })
            })
            .collect();
        
        handles
            .into_iter()
            .map(|handle| handle.join().unwrap())
            .collect()
    }
}

// 6. 命令行界面
pub struct CLI {
    state: Arc<AppState>,
}

impl CLI {
    pub fn new(state: Arc<AppState>) -> Self {
        Self { state }
    }
    
    pub fn run(&self) -> Result<()> {
        println!("=== Rust基础综合项目 - 任务管理系统 ===");
        println!("输入 'help' 查看可用命令");
        
        loop {
            print!("> ");
            io::stdout().flush()?;
            
            let mut input = String::new();
            io::stdin().read_line(&mut input)?;
            let input = input.trim();
            
            if input.is_empty() {
                continue;
            }
            
            match self.handle_command(input) {
                Ok(should_continue) => {
                    if !should_continue {
                        break;
                    }
                }
                Err(e) => {
                    error!("命令执行错误: {}", e);
                    println!("错误: {}", e);
                }
            }
        }
        
        Ok(())
    }
    
    fn handle_command(&self, input: &str) -> Result<bool> {
        let parts: Vec<&str> = input.split_whitespace().collect();
        if parts.is_empty() {
            return Ok(true);
        }
        
        match parts[0] {
            "help" => {
                self.show_help();
            }
            "create_user" => {
                if parts.len() < 3 {
                    println!("用法: create_user <name> <email>");
                    return Ok(true);
                }
                let id = self.state.create_user(parts[1].to_string(), parts[2].to_string())?;
                println!("创建用户成功，ID: {}", id);
            }
            "create_task" => {
                if parts.len() < 3 {
                    println!("用法: create_task <title> <description>");
                    return Ok(true);
                }
                let title = parts[1].to_string();
                let description = parts[2..].join(" ");
                let id = self.state.create_task(title, description)?;
                println!("创建任务成功，ID: {}", id);
            }
            "assign_task" => {
                if parts.len() < 3 {
                    println!("用法: assign_task <task_id> <user_id>");
                    return Ok(true);
                }
                let task_id: u64 = parts[1].parse().map_err(|_| {
                    AppError::Business("无效的任务ID".to_string())
                })?;
                let user_id: u64 = parts[2].parse().map_err(|_| {
                    AppError::Business("无效的用户ID".to_string())
                })?;
                self.state.assign_task(task_id, user_id)?;
                println!("任务分配成功");
            }
            "complete_task" => {
                if parts.len() < 2 {
                    println!("用法: complete_task <task_id>");
                    return Ok(true);
                }
                let task_id: u64 = parts[1].parse().map_err(|_| {
                    AppError::Business("无效的任务ID".to_string())
                })?;
                self.state.complete_task(task_id)?;
                println!("任务完成");
            }
            "list_users" => {
                self.list_users();
            }
            "list_tasks" => {
                self.list_tasks();
            }
            "stats" => {
                self.show_stats();
            }
            "save" => {
                let filename = if parts.len() > 1 { parts[1] } else { "data.json" };
                self.state.save_to_file(filename)?;
                println!("数据已保存到 {}", filename);
            }
            "load" => {
                let filename = if parts.len() > 1 { parts[1] } else { "data.json" };
                self.state.load_from_file(filename)?;
                println!("数据已从 {} 加载", filename);
            }
            "quit" | "exit" => {
                println!("再见！");
                return Ok(false);
            }
            _ => {
                println!("未知命令: {}，输入 'help' 查看可用命令", parts[0]);
            }
        }
        
        Ok(true)
    }
    
    fn show_help(&self) {
        println!("可用命令:");
        println!("  create_user <name> <email>     - 创建用户");
        println!("  create_task <title> <desc>     - 创建任务");
        println!("  assign_task <task_id> <user_id> - 分配任务");
        println!("  complete_task <task_id>        - 完成任务");
        println!("  list_users                     - 列出所有用户");
        println!("  list_tasks                     - 列出所有任务");
        println!("  stats                          - 显示统计信息");
        println!("  save [filename]                - 保存数据");
        println!("  load [filename]                - 加载数据");
        println!("  help                           - 显示帮助");
        println!("  quit/exit                      - 退出程序");
    }
    
    fn list_users(&self) {
        let users = self.state.users.lock().unwrap();
        if users.is_empty() {
            println!("没有用户");
            return;
        }
        
        println!("用户列表:");
        for user in users.values() {
            println!("  ID: {}, 姓名: {}, 邮箱: {}, 状态: {}", 
                     user.id, user.name, user.email, 
                     if user.is_active { "活跃" } else { "非活跃" });
        }
    }
    
    fn list_tasks(&self) {
        let tasks = self.state.tasks.lock().unwrap();
        if tasks.is_empty() {
            println!("没有任务");
            return;
        }
        
        println!("任务列表:");
        for task in tasks.values() {
            println!("  ID: {}, 标题: {}, 状态: {:?}, 分配给: {:?}, 优先级: {:?}", 
                     task.id, task.title, task.status, task.assigned_to, task.priority);
        }
    }
    
    fn show_stats(&self) {
        let stats = self.state.get_stats();
        println!("系统统计:");
        println!("  总用户数: {}", stats.total_users);
        println!("  活跃用户数: {}", stats.active_users);
        println!("  总任务数: {}", stats.total_tasks);
        println!("  已完成任务数: {}", stats.completed_tasks);
        println!("  运行时间: {:?}", stats.uptime);
    }
}

// 7. 主函数
fn main() -> Result<()> {
    // 初始化日志
    env_logger::init();
    
    info!("应用程序启动");
    
    // 创建应用状态
    let state = Arc::new(AppState::new());
    
    // 启动后台任务处理器
    let processor = TaskProcessor::new(Arc::clone(&state));
    let _background_handle = processor.start_background_processing();
    
    // 创建一些示例数据
    create_sample_data(&state)?;
    
    // 启动CLI
    let cli = CLI::new(state);
    cli.run()?;
    
    info!("应用程序结束");
    Ok(())
}

// 创建示例数据
fn create_sample_data(state: &AppState) -> Result<()> {
    info!("创建示例数据");
    
    // 创建用户
    let user1_id = state.create_user("张三".to_string(), "zhangsan@example.com".to_string())?;
    let user2_id = state.create_user("李四".to_string(), "lisi@example.com".to_string())?;
    let user3_id = state.create_user("王五".to_string(), "wangwu@example.com".to_string())?;
    
    // 添加用户元数据
    state.update_user(user1_id, |user| {
        user.add_metadata("department".to_string(), "开发部".to_string());
        user.add_metadata("role".to_string(), "高级工程师".to_string());
        Ok(())
    })?;
    
    // 创建任务
    let task1_id = state.create_task(
        "实现用户认证系统".to_string(),
        "设计并实现完整的用户认证和授权系统".to_string(),
    )?;
    
    let task2_id = state.create_task(
        "优化数据库查询".to_string(),
        "分析并优化慢查询，提升系统性能".to_string(),
    )?;
    
    let task3_id = state.create_task(
        "编写API文档".to_string(),
        "为所有REST API端点编写详细文档".to_string(),
    )?;
    
    // 设置任务优先级
    state.update_task(task1_id, |task| {
        task.set_priority(Priority::High);
        Ok(())
    })?;
    
    state.update_task(task2_id, |task| {
        task.set_priority(Priority::Critical);
        Ok(())
    })?;
    
    // 分配任务
    state.assign_task(task1_id, user1_id)?;
    state.assign_task(task2_id, user2_id)?;
    state.assign_task(task3_id, user3_id)?;
    
    // 完成一个任务
    state.complete_task(task3_id)?;
    
    info!("示例数据创建完成");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_user_creation() {
        let state = AppState::new();
        let user_id = state.create_user("测试用户".to_string(), "test@example.com".to_string()).unwrap();
        
        let user = state.get_user(user_id).unwrap();
        assert_eq!(user.name, "测试用户");
        assert_eq!(user.email, "test@example.com");
        assert!(user.is_active);
    }
    
    #[test]
    fn test_task_creation_and_assignment() {
        let state = AppState::new();
        let user_id = state.create_user("测试用户".to_string(), "test@example.com".to_string()).unwrap();
        let task_id = state.create_task("测试任务".to_string(), "这是一个测试任务".to_string()).unwrap();
        
        state.assign_task(task_id, user_id).unwrap();
        
        let task = state.get_task(task_id).unwrap();
        assert_eq!(task.assigned_to, Some(user_id));
        
        let user_tasks = state.get_user_tasks(user_id);
        assert_eq!(user_tasks.len(), 1);
        assert_eq!(user_tasks[0].id, task_id);
    }
    
    #[test]
    fn test_task_completion() {
        let state = AppState::new();
        let task_id = state.create_task("测试任务".to_string(), "这是一个测试任务".to_string()).unwrap();
        
        state.complete_task(task_id).unwrap();
        
        let task = state.get_task(task_id).unwrap();
        assert_eq!(task.status, TaskStatus::Completed);
        
        let stats = state.get_stats();
        assert_eq!(stats.completed_tasks, 1);
    }
    
    #[test]
    fn test_user_validation() {
        let user = User::new(1, "".to_string(), "invalid-email".to_string());
        assert!(user.validate().is_err());
        
        let user = User::new(1, "有效用户".to_string(), "valid@example.com".to_string());
        assert!(user.validate().is_ok());
    }
    
    #[test]
    fn test_task_priority_filtering() {
        let state = AppState::new();
        let task1_id = state.create_task("普通任务".to_string(), "描述".to_string()).unwrap();
        let task2_id = state.create_task("高优先级任务".to_string(), "描述".to_string()).unwrap();
        
        state.update_task(task2_id, |task| {
            task.set_priority(Priority::High);
            Ok(())
        }).unwrap();
        
        let high_priority_tasks = state.get_high_priority_tasks();
        assert_eq!(high_priority_tasks.len(), 1);
        assert_eq!(high_priority_tasks[0].id, task2_id);
    }
}