//! 基础PLC控制系统示例
//! 
//! 这个示例展示了一个简单的PLC控制逻辑，包括：
//! - 数字输入/输出处理
//! - 定时器功能
//! - 基本的逻辑控制

use std::time::{Duration, Instant};
use std::collections::HashMap;

/// PLC输入状态
#[derive(Debug, Clone)]
struct PlcInputs {
    digital_inputs: HashMap<u8, bool>,
    analog_inputs: HashMap<u8, f32>,
}

/// PLC输出状态
#[derive(Debug, Clone)]
struct PlcOutputs {
    digital_outputs: HashMap<u8, bool>,
    analog_outputs: HashMap<u8, f32>,
}

/// 定时器结构
#[derive(Debug)]
struct Timer {
    start_time: Option<Instant>,
    duration: Duration,
    is_running: bool,
}

impl Timer {
    fn new(duration_ms: u64) -> Self {
        Self {
            start_time: None,
            duration: Duration::from_millis(duration_ms),
            is_running: false,
        }
    }
    
    fn start(&mut self) {
        self.start_time = Some(Instant::now());
        self.is_running = true;
    }
    
    fn is_done(&self) -> bool {
        if let Some(start) = self.start_time {
            start.elapsed() >= self.duration
        } else {
            false
        }
    }
    
    fn reset(&mut self) {
        self.start_time = None;
        self.is_running = false;
    }
}

/// 基础PLC控制器
struct BasicPlc {
    inputs: PlcInputs,
    outputs: PlcOutputs,
    timers: HashMap<u8, Timer>,
}

impl BasicPlc {
    fn new() -> Self {
        Self {
            inputs: PlcInputs {
                digital_inputs: HashMap::new(),
                analog_inputs: HashMap::new(),
            },
            outputs: PlcOutputs {
                digital_outputs: HashMap::new(),
                analog_outputs: HashMap::new(),
            },
            timers: HashMap::new(),
        }
    }
    
    /// 更新输入状态
    fn update_inputs(&mut self, digital: HashMap<u8, bool>, analog: HashMap<u8, f32>) {
        self.inputs.digital_inputs = digital;
        self.inputs.analog_inputs = analog;
    }
    
    /// 执行控制逻辑
    fn execute_logic(&mut self) {
        // 示例控制逻辑：启动按钮控制
        if let Some(&start_button) = self.inputs.digital_inputs.get(&0) {
            if start_button {
                // 启动定时器
                if !self.timers.contains_key(&0) {
                    self.timers.insert(0, Timer::new(5000)); // 5秒定时器
                }
                self.timers.get_mut(&0).unwrap().start();
                
                // 设置输出
                self.outputs.digital_outputs.insert(0, true); // 启动指示灯
            }
        }
        
        // 检查定时器状态
        if let Some(timer) = self.timers.get(&0) {
            if timer.is_done() {
                self.outputs.digital_outputs.insert(1, true); // 完成指示灯
            }
        }
        
        // 停止按钮逻辑
        if let Some(&stop_button) = self.inputs.digital_inputs.get(&1) {
            if stop_button {
                // 重置所有输出
                self.outputs.digital_outputs.clear();
                // 重置定时器
                if let Some(timer) = self.timers.get_mut(&0) {
                    timer.reset();
                }
            }
        }
    }
    
    /// 获取输出状态
    fn get_outputs(&self) -> &PlcOutputs {
        &self.outputs
    }
}

fn main() {
    println!("基础PLC控制系统示例");
    
    let mut plc = BasicPlc::new();
    
    // 模拟运行循环
    for cycle in 0..10 {
        println!("\n=== 循环 {} ===", cycle);
        
        // 模拟输入变化
        let mut digital_inputs = HashMap::new();
        let analog_inputs = HashMap::new();
        
        match cycle {
            2 => {
                digital_inputs.insert(0, true); // 启动按钮按下
                println!("启动按钮按下");
            },
            7 => {
                digital_inputs.insert(1, true); // 停止按钮按下
                println!("停止按钮按下");
            },
            _ => {}
        }
        
        // 更新输入并执行逻辑
        plc.update_inputs(digital_inputs, analog_inputs);
        plc.execute_logic();
        
        // 显示输出状态
        let outputs = plc.get_outputs();
        println!("数字输出: {:?}", outputs.digital_outputs);
        
        // 模拟扫描周期
        std::thread::sleep(Duration::from_millis(100));
    }
    
    println!("\nPLC示例运行完成");
}