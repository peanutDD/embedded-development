//! 安全控制器示例
//! 
//! 这个示例展示了工业安全控制系统的基本功能：
//! - 安全输入监控
//! - 安全逻辑处理
//! - 故障诊断
//! - 紧急停止处理
//! - 安全完整性等级(SIL)管理

use std::collections::HashMap;
use std::time::{Duration, Instant, SystemTime};
use std::sync::{Arc, Mutex};
use std::thread;

/// 安全完整性等级
#[derive(Debug, Clone, PartialEq, PartialOrd)]
enum SafetyIntegrityLevel {
    SIL1,
    SIL2,
    SIL3,
    SIL4,
}

/// 安全输入类型
#[derive(Debug, Clone, PartialEq, Hash, Eq)]
enum SafetyInputType {
    EmergencyStop,
    LightCurtain,
    SafetyDoor,
    PressureMat,
    TwoHandControl,
    SafetyRelay,
}

/// 安全输入状态
#[derive(Debug, Clone)]
struct SafetyInput {
    id: u32,
    name: String,
    input_type: SafetyInputType,
    current_state: bool,
    previous_state: bool,
    fault_detected: bool,
    test_required: bool,
    last_test_time: Option<Instant>,
    sil_level: SafetyIntegrityLevel,
}

/// 安全输出状态
#[derive(Debug, Clone)]
struct SafetyOutput {
    id: u32,
    name: String,
    current_state: bool,
    fault_detected: bool,
    sil_level: SafetyIntegrityLevel,
}

/// 故障类型
#[derive(Debug, Clone, PartialEq)]
enum FaultType {
    InputStuck,
    OutputFault,
    CommunicationError,
    SelfTestFailure,
    PowerSupplyFault,
    WatchdogTimeout,
}

/// 故障记录
#[derive(Debug, Clone)]
struct FaultRecord {
    id: u32,
    fault_type: FaultType,
    description: String,
    timestamp: SystemTime,
    severity: FaultSeverity,
    acknowledged: bool,
}

/// 故障严重程度
#[derive(Debug, Clone, PartialEq, PartialOrd)]
enum FaultSeverity {
    Info,
    Warning,
    Error,
    Critical,
}

/// 系统状态
#[derive(Debug, Clone, PartialEq)]
enum SystemState {
    Safe,
    Operational,
    Warning,
    Fault,
    EmergencyStop,
}

/// 安全控制器主结构
struct SafetyController {
    inputs: HashMap<u32, SafetyInput>,
    outputs: HashMap<u32, SafetyOutput>,
    faults: Vec<FaultRecord>,
    system_state: SystemState,
    emergency_stop_active: bool,
    watchdog_timer: Instant,
    self_test_interval: Duration,
    last_self_test: Instant,
    fault_counter: u32,
}

impl SafetyController {
    fn new() -> Self {
        Self {
            inputs: HashMap::new(),
            outputs: HashMap::new(),
            faults: Vec::new(),
            system_state: SystemState::Safe,
            emergency_stop_active: false,
            watchdog_timer: Instant::now(),
            self_test_interval: Duration::from_secs(60), // 1分钟自检间隔
            last_self_test: Instant::now(),
            fault_counter: 0,
        }
    }
    
    /// 添加安全输入
    fn add_safety_input(&mut self, input: SafetyInput) {
        self.inputs.insert(input.id, input);
    }
    
    /// 添加安全输出
    fn add_safety_output(&mut self, output: SafetyOutput) {
        self.outputs.insert(output.id, output);
    }
    
    /// 更新输入状态
    fn update_input_state(&mut self, input_id: u32, new_state: bool) {
        if let Some(input) = self.inputs.get_mut(&input_id) {
            input.previous_state = input.current_state;
            input.current_state = new_state;
            
            // 检测状态变化
            if input.previous_state != input.current_state {
                println!("安全输入 {} 状态变化: {} -> {}", 
                        input.name, input.previous_state, input.current_state);
            }
        }
    }
    
    /// 执行安全逻辑
    fn execute_safety_logic(&mut self) {
        // 重置看门狗
        self.watchdog_timer = Instant::now();
        
        // 检查紧急停止
        self.check_emergency_stop();
        
        // 检查安全输入
        self.check_safety_inputs();
        
        // 执行自检
        if self.last_self_test.elapsed() >= self.self_test_interval {
            self.perform_self_test();
            self.last_self_test = Instant::now();
        }
        
        // 更新系统状态
        self.update_system_state();
        
        // 更新安全输出
        self.update_safety_outputs();
    }
    
    /// 检查紧急停止
    fn check_emergency_stop(&mut self) {
        let mut emergency_stop_triggered = false;
        
        for input in self.inputs.values() {
            if input.input_type == SafetyInputType::EmergencyStop {
                if !input.current_state { // 紧急停止按钮被按下（常闭触点）
                    emergency_stop_triggered = true;
                    break;
                }
            }
        }
        
        if emergency_stop_triggered && !self.emergency_stop_active {
            self.emergency_stop_active = true;
            self.system_state = SystemState::EmergencyStop;
            self.log_fault(FaultType::InputStuck, 
                          "紧急停止被触发".to_string(), 
                          FaultSeverity::Critical);
            println!("⚠️  紧急停止被触发！");
        } else if !emergency_stop_triggered && self.emergency_stop_active {
            println!("紧急停止已复位");
        }
    }
    
    /// 检查安全输入
    fn check_safety_inputs(&mut self) {
        for input in self.inputs.values_mut() {
            // 检查光幕
            if input.input_type == SafetyInputType::LightCurtain {
                if !input.current_state {
                    self.log_fault(FaultType::InputStuck, 
                                  format!("光幕 {} 被遮挡", input.name), 
                                  FaultSeverity::Warning);
                }
            }
            
            // 检查安全门
            if input.input_type == SafetyInputType::SafetyDoor {
                if !input.current_state {
                    self.log_fault(FaultType::InputStuck, 
                                  format!("安全门 {} 未关闭", input.name), 
                                  FaultSeverity::Error);
                }
            }
            
            // 检查双手控制
            if input.input_type == SafetyInputType::TwoHandControl {
                // 双手控制需要特殊逻辑检查同步性
                if input.current_state {
                    // 这里应该检查两个按钮是否同时按下
                    // 简化实现
                }
            }
            
            // 检查是否需要测试
            if input.test_required {
                if let Some(last_test) = input.last_test_time {
                    if last_test.elapsed() > Duration::from_secs(3600) { // 1小时测试间隔
                        self.log_fault(FaultType::SelfTestFailure, 
                                      format!("输入 {} 需要测试", input.name), 
                                      FaultSeverity::Warning);
                    }
                }
            }
        }
    }
    
    /// 执行自检
    fn perform_self_test(&mut self) {
        println!("执行系统自检...");
        
        // 检查输入通道
        for input in self.inputs.values_mut() {
            // 模拟输入测试
            if input.sil_level >= SafetyIntegrityLevel::SIL2 {
                // 高SIL等级需要更严格的测试
                if !self.test_input_channel(input.id) {
                    input.fault_detected = true;
                    self.log_fault(FaultType::SelfTestFailure, 
                                  format!("输入 {} 自检失败", input.name), 
                                  FaultSeverity::Error);
                }
            }
            input.last_test_time = Some(Instant::now());
        }
        
        // 检查输出通道
        for output in self.outputs.values_mut() {
            if !self.test_output_channel(output.id) {
                output.fault_detected = true;
                self.log_fault(FaultType::OutputFault, 
                              format!("输出 {} 自检失败", output.name), 
                              FaultSeverity::Error);
            }
        }
        
        println!("系统自检完成");
    }
    
    /// 测试输入通道
    fn test_input_channel(&self, _input_id: u32) -> bool {
        // 模拟输入通道测试
        // 实际实现中会进行硬件级别的测试
        true // 假设测试通过
    }
    
    /// 测试输出通道
    fn test_output_channel(&self, _output_id: u32) -> bool {
        // 模拟输出通道测试
        // 实际实现中会进行硬件级别的测试
        true // 假设测试通过
    }
    
    /// 更新系统状态
    fn update_system_state(&mut self) {
        if self.emergency_stop_active {
            self.system_state = SystemState::EmergencyStop;
            return;
        }
        
        let critical_faults = self.faults.iter()
            .filter(|f| f.severity == FaultSeverity::Critical && !f.acknowledged)
            .count();
        
        let error_faults = self.faults.iter()
            .filter(|f| f.severity == FaultSeverity::Error && !f.acknowledged)
            .count();
        
        let warning_faults = self.faults.iter()
            .filter(|f| f.severity == FaultSeverity::Warning && !f.acknowledged)
            .count();
        
        self.system_state = if critical_faults > 0 {
            SystemState::Fault
        } else if error_faults > 0 {
            SystemState::Fault
        } else if warning_faults > 0 {
            SystemState::Warning
        } else {
            // 检查所有安全输入是否正常
            let all_inputs_safe = self.inputs.values()
                .all(|input| match input.input_type {
                    SafetyInputType::EmergencyStop => input.current_state,
                    SafetyInputType::SafetyDoor => input.current_state,
                    SafetyInputType::LightCurtain => input.current_state,
                    _ => true,
                });
            
            if all_inputs_safe {
                SystemState::Operational
            } else {
                SystemState::Safe
            }
        };
    }
    
    /// 更新安全输出
    fn update_safety_outputs(&mut self) {
        let safe_to_operate = matches!(self.system_state, SystemState::Operational);
        
        for output in self.outputs.values_mut() {
            if output.fault_detected {
                output.current_state = false; // 故障时强制关闭输出
            } else {
                output.current_state = safe_to_operate;
            }
        }
    }
    
    /// 记录故障
    fn log_fault(&mut self, fault_type: FaultType, description: String, severity: FaultSeverity) {
        self.fault_counter += 1;
        let fault = FaultRecord {
            id: self.fault_counter,
            fault_type,
            description: description.clone(),
            timestamp: SystemTime::now(),
            severity,
            acknowledged: false,
        };
        
        self.faults.push(fault);
        println!("故障记录: {}", description);
    }
    
    /// 确认故障
    fn acknowledge_fault(&mut self, fault_id: u32) {
        if let Some(fault) = self.faults.iter_mut().find(|f| f.id == fault_id) {
            fault.acknowledged = true;
            println!("故障 {} 已确认", fault_id);
        }
    }
    
    /// 获取系统状态
    fn get_system_status(&self) -> SystemStatus {
        let active_faults = self.faults.iter()
            .filter(|f| !f.acknowledged)
            .count();
        
        SystemStatus {
            state: self.system_state.clone(),
            emergency_stop_active: self.emergency_stop_active,
            active_fault_count: active_faults,
            total_inputs: self.inputs.len(),
            total_outputs: self.outputs.len(),
            uptime: self.watchdog_timer.elapsed(),
        }
    }
}

/// 系统状态信息
#[derive(Debug)]
struct SystemStatus {
    state: SystemState,
    emergency_stop_active: bool,
    active_fault_count: usize,
    total_inputs: usize,
    total_outputs: usize,
    uptime: Duration,
}

fn main() {
    println!("安全控制器示例");
    
    let mut safety_controller = SafetyController::new();
    
    // 添加安全输入
    safety_controller.add_safety_input(SafetyInput {
        id: 1,
        name: "紧急停止按钮-1".to_string(),
        input_type: SafetyInputType::EmergencyStop,
        current_state: true, // 正常状态（未按下）
        previous_state: true,
        fault_detected: false,
        test_required: true,
        last_test_time: None,
        sil_level: SafetyIntegrityLevel::SIL3,
    });
    
    safety_controller.add_safety_input(SafetyInput {
        id: 2,
        name: "安全门开关".to_string(),
        input_type: SafetyInputType::SafetyDoor,
        current_state: true, // 门关闭状态
        previous_state: true,
        fault_detected: false,
        test_required: true,
        last_test_time: None,
        sil_level: SafetyIntegrityLevel::SIL2,
    });
    
    safety_controller.add_safety_input(SafetyInput {
        id: 3,
        name: "光幕传感器".to_string(),
        input_type: SafetyInputType::LightCurtain,
        current_state: true, // 光幕未遮挡
        previous_state: true,
        fault_detected: false,
        test_required: true,
        last_test_time: None,
        sil_level: SafetyIntegrityLevel::SIL2,
    });
    
    // 添加安全输出
    safety_controller.add_safety_output(SafetyOutput {
        id: 1,
        name: "主电机使能".to_string(),
        current_state: false,
        fault_detected: false,
        sil_level: SafetyIntegrityLevel::SIL2,
    });
    
    safety_controller.add_safety_output(SafetyOutput {
        id: 2,
        name: "安全继电器".to_string(),
        current_state: false,
        fault_detected: false,
        sil_level: SafetyIntegrityLevel::SIL3,
    });
    
    println!("\n开始安全控制循环...");
    
    // 模拟运行循环
    for cycle in 0..20 {
        println!("\n=== 循环 {} ===", cycle);
        
        // 模拟输入变化
        match cycle {
            5 => {
                // 模拟光幕被遮挡
                safety_controller.update_input_state(3, false);
                println!("模拟: 光幕被遮挡");
            },
            8 => {
                // 光幕恢复
                safety_controller.update_input_state(3, true);
                println!("模拟: 光幕恢复");
            },
            12 => {
                // 模拟紧急停止
                safety_controller.update_input_state(1, false);
                println!("模拟: 紧急停止按下");
            },
            15 => {
                // 紧急停止复位
                safety_controller.update_input_state(1, true);
                println!("模拟: 紧急停止复位");
            },
            _ => {}
        }
        
        // 执行安全逻辑
        safety_controller.execute_safety_logic();
        
        // 显示系统状态
        let status = safety_controller.get_system_status();
        println!("系统状态: {:?}", status.state);
        println!("活跃故障数: {}", status.active_fault_count);
        
        // 确认一些故障（模拟操作员确认）
        if cycle == 10 && status.active_fault_count > 0 {
            safety_controller.acknowledge_fault(1);
        }
        
        // 模拟扫描周期
        thread::sleep(Duration::from_millis(100));
    }
    
    // 显示最终状态
    let final_status = safety_controller.get_system_status();
    println!("\n=== 最终系统状态 ===");
    println!("{:#?}", final_status);
    
    println!("\n安全控制器示例完成");
}