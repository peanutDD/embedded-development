# PLC基础与工业控制器

本文档介绍基于STM32F4的工业控制器(PLC)基础知识，包括工业控制系统架构、I/O处理、逻辑控制和通信协议等核心概念。

## 工业控制系统概述

### PLC系统架构
```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   现场设备      │    │   工业控制器    │    │   上位机系统    │
│                 │    │                 │    │                 │
│ • 传感器        │◄──►│ • CPU模块       │◄──►│ • SCADA系统     │
│ • 执行器        │    │ • I/O模块       │    │ • HMI界面       │
│ • 变送器        │    │ • 通信模块      │    │ • 数据库        │
│ • 阀门/电机     │    │ • 电源模块      │    │ • 报表系统      │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

### STM32F4工业控制器特性
- **高性能处理器**: ARM Cortex-M4, 168MHz
- **丰富的I/O接口**: GPIO, ADC, DAC, PWM, 通信接口
- **实时性能**: 中断响应时间 < 1μs
- **工业级可靠性**: -40°C ~ +85°C工作温度
- **低功耗设计**: 多种省电模式

## I/O系统设计

### 数字输入(DI)模块
```rust
use stm32f4xx_hal::{
    prelude::*,
    gpio::{Input, PullUp, PullDown},
    stm32,
};

/// 数字输入通道
#[derive(Clone, Copy)]
pub struct DigitalInput {
    pub channel: u8,
    pub state: bool,
    pub filter_count: u8,
    pub debounce_time: u16,    // ms
    pub invert: bool,
    pub alarm_enable: bool,
}

/// 数字输入模块
pub struct DigitalInputModule {
    inputs: [DigitalInput; 16],
    filter_counters: [u8; 16],
    last_states: [bool; 16],
    debounce_timers: [u16; 16],
}

impl DigitalInputModule {
    pub fn new() -> Self {
        Self {
            inputs: [DigitalInput {
                channel: 0,
                state: false,
                filter_count: 3,
                debounce_time: 10,
                invert: false,
                alarm_enable: false,
            }; 16],
            filter_counters: [0; 16],
            last_states: [false; 16],
            debounce_timers: [0; 16],
        }
    }
    
    /// 读取数字输入
    pub fn read_inputs(&mut self, gpio_states: &[bool; 16]) {
        for i in 0..16 {
            let raw_state = gpio_states[i] ^ self.inputs[i].invert;
            
            // 数字滤波
            if raw_state == self.last_states[i] {
                if self.filter_counters[i] < self.inputs[i].filter_count {
                    self.filter_counters[i] += 1;
                } else {
                    // 防抖处理
                    if self.debounce_timers[i] == 0 {
                        self.inputs[i].state = raw_state;
                        self.debounce_timers[i] = self.inputs[i].debounce_time;
                    }
                }
            } else {
                self.filter_counters[i] = 0;
                self.last_states[i] = raw_state;
            }
        }
    }
    
    /// 更新防抖定时器
    pub fn update_debounce_timers(&mut self) {
        for timer in &mut self.debounce_timers {
            if *timer > 0 {
                *timer -= 1;
            }
        }
    }
    
    /// 获取输入状态
    pub fn get_input(&self, channel: u8) -> bool {
        if channel < 16 {
            self.inputs[channel as usize].state
        } else {
            false
        }
    }
    
    /// 配置输入通道
    pub fn configure_input(&mut self, channel: u8, config: DigitalInput) {
        if channel < 16 {
            self.inputs[channel as usize] = config;
        }
    }
}
```

### 数字输出(DO)模块
```rust
/// 数字输出通道
#[derive(Clone, Copy)]
pub struct DigitalOutput {
    pub channel: u8,
    pub state: bool,
    pub invert: bool,
    pub pulse_mode: bool,
    pub pulse_width: u16,      // ms
    pub fault_state: bool,     // 故障时的安全状态
}

/// 数字输出模块
pub struct DigitalOutputModule {
    outputs: [DigitalOutput; 16],
    pulse_timers: [u16; 16],
    fault_mode: bool,
}

impl DigitalOutputModule {
    pub fn new() -> Self {
        Self {
            outputs: [DigitalOutput {
                channel: 0,
                state: false,
                invert: false,
                pulse_mode: false,
                pulse_width: 100,
                fault_state: false,
            }; 16],
            pulse_timers: [0; 16],
            fault_mode: false,
        }
    }
    
    /// 设置输出状态
    pub fn set_output(&mut self, channel: u8, state: bool) {
        if channel < 16 {
            let idx = channel as usize;
            
            if self.outputs[idx].pulse_mode && state {
                // 脉冲模式
                self.pulse_timers[idx] = self.outputs[idx].pulse_width;
                self.outputs[idx].state = true;
            } else {
                self.outputs[idx].state = state;
            }
        }
    }
    
    /// 更新脉冲定时器
    pub fn update_pulse_timers(&mut self) {
        for i in 0..16 {
            if self.pulse_timers[i] > 0 {
                self.pulse_timers[i] -= 1;
                if self.pulse_timers[i] == 0 {
                    self.outputs[i].state = false;
                }
            }
        }
    }
    
    /// 获取物理输出状态
    pub fn get_physical_outputs(&self) -> [bool; 16] {
        let mut physical_states = [false; 16];
        
        for i in 0..16 {
            if self.fault_mode {
                physical_states[i] = self.outputs[i].fault_state ^ self.outputs[i].invert;
            } else {
                physical_states[i] = self.outputs[i].state ^ self.outputs[i].invert;
            }
        }
        
        physical_states
    }
    
    /// 设置故障模式
    pub fn set_fault_mode(&mut self, fault: bool) {
        self.fault_mode = fault;
    }
}
```

### 模拟输入(AI)模块
```rust
use micromath::F32Ext;

/// 模拟输入通道
#[derive(Clone, Copy)]
pub struct AnalogInput {
    pub channel: u8,
    pub raw_value: u16,
    pub scaled_value: f32,
    pub min_scale: f32,
    pub max_scale: f32,
    pub offset: f32,
    pub gain: f32,
    pub filter_enable: bool,
    pub filter_constant: f32,
    pub alarm_high: f32,
    pub alarm_low: f32,
    pub alarm_enable: bool,
}

/// 模拟输入模块
pub struct AnalogInputModule {
    inputs: [AnalogInput; 8],
    filter_values: [f32; 8],
    alarm_states: [bool; 8],
}

impl AnalogInputModule {
    pub fn new() -> Self {
        Self {
            inputs: [AnalogInput {
                channel: 0,
                raw_value: 0,
                scaled_value: 0.0,
                min_scale: 0.0,
                max_scale: 100.0,
                offset: 0.0,
                gain: 1.0,
                filter_enable: true,
                filter_constant: 0.1,
                alarm_high: 90.0,
                alarm_low: 10.0,
                alarm_enable: false,
            }; 8],
            filter_values: [0.0; 8],
            alarm_states: [false; 8],
        }
    }
    
    /// 更新模拟输入
    pub fn update_inputs(&mut self, adc_values: &[u16; 8]) {
        for i in 0..8 {
            self.inputs[i].raw_value = adc_values[i];
            
            // 线性标定
            let normalized = (adc_values[i] as f32) / 4095.0;
            let range = self.inputs[i].max_scale - self.inputs[i].min_scale;
            let scaled = self.inputs[i].min_scale + normalized * range;
            let calibrated = (scaled + self.inputs[i].offset) * self.inputs[i].gain;
            
            // 数字滤波
            if self.inputs[i].filter_enable {
                let alpha = self.inputs[i].filter_constant;
                self.filter_values[i] = alpha * calibrated + (1.0 - alpha) * self.filter_values[i];
                self.inputs[i].scaled_value = self.filter_values[i];
            } else {
                self.inputs[i].scaled_value = calibrated;
            }
            
            // 报警检查
            if self.inputs[i].alarm_enable {
                self.alarm_states[i] = 
                    self.inputs[i].scaled_value > self.inputs[i].alarm_high ||
                    self.inputs[i].scaled_value < self.inputs[i].alarm_low;
            }
        }
    }
    
    /// 获取标定值
    pub fn get_scaled_value(&self, channel: u8) -> f32 {
        if channel < 8 {
            self.inputs[channel as usize].scaled_value
        } else {
            0.0
        }
    }
    
    /// 获取报警状态
    pub fn get_alarm_state(&self, channel: u8) -> bool {
        if channel < 8 {
            self.alarm_states[channel as usize]
        } else {
            false
        }
    }
}
```

### 模拟输出(AO)模块
```rust
/// 模拟输出通道
#[derive(Clone, Copy)]
pub struct AnalogOutput {
    pub channel: u8,
    pub setpoint: f32,
    pub output_value: u16,
    pub min_scale: f32,
    pub max_scale: f32,
    pub offset: f32,
    pub gain: f32,
    pub rate_limit: f32,       // 变化率限制
    pub fault_value: f32,      // 故障时输出值
}

/// 模拟输出模块
pub struct AnalogOutputModule {
    outputs: [AnalogOutput; 4],
    last_values: [f32; 4],
    fault_mode: bool,
}

impl AnalogOutputModule {
    pub fn new() -> Self {
        Self {
            outputs: [AnalogOutput {
                channel: 0,
                setpoint: 0.0,
                output_value: 0,
                min_scale: 0.0,
                max_scale: 100.0,
                offset: 0.0,
                gain: 1.0,
                rate_limit: 10.0,  // 10%/s
                fault_value: 0.0,
            }; 4],
            last_values: [0.0; 4],
            fault_mode: false,
        }
    }
    
    /// 设置输出值
    pub fn set_output(&mut self, channel: u8, value: f32) {
        if channel < 4 {
            self.outputs[channel as usize].setpoint = value;
        }
    }
    
    /// 更新输出
    pub fn update_outputs(&mut self, dt: f32) {
        for i in 0..4 {
            let target_value = if self.fault_mode {
                self.outputs[i].fault_value
            } else {
                self.outputs[i].setpoint
            };
            
            // 变化率限制
            let max_change = self.outputs[i].rate_limit * dt;
            let change = target_value - self.last_values[i];
            let limited_change = if change.abs() > max_change {
                if change > 0.0 { max_change } else { -max_change }
            } else {
                change
            };
            
            let new_value = self.last_values[i] + limited_change;
            self.last_values[i] = new_value;
            
            // 标定和限制
            let calibrated = (new_value * self.outputs[i].gain + self.outputs[i].offset)
                .max(self.outputs[i].min_scale)
                .min(self.outputs[i].max_scale);
            
            // 转换为DAC值
            let normalized = (calibrated - self.outputs[i].min_scale) / 
                           (self.outputs[i].max_scale - self.outputs[i].min_scale);
            self.outputs[i].output_value = (normalized * 4095.0) as u16;
        }
    }
    
    /// 获取DAC输出值
    pub fn get_dac_values(&self) -> [u16; 4] {
        [
            self.outputs[0].output_value,
            self.outputs[1].output_value,
            self.outputs[2].output_value,
            self.outputs[3].output_value,
        ]
    }
}
```

## 逻辑控制引擎

### 梯形图逻辑
```rust
/// 梯形图指令类型
#[derive(Clone, Copy, PartialEq)]
pub enum LadderInstruction {
    // 触点指令
    NormallyOpen(u16),      // 常开触点
    NormallyClosed(u16),    // 常闭触点
    RisingEdge(u16),        // 上升沿
    FallingEdge(u16),       // 下降沿
    
    // 线圈指令
    Coil(u16),              // 输出线圈
    Set(u16),               // 置位
    Reset(u16),             // 复位
    
    // 逻辑指令
    And,                    // 与
    Or,                     // 或
    Not,                    // 非
    
    // 定时器指令
    Timer(u16, u32),        // 定时器(编号, 设定值)
    Counter(u16, u32),      // 计数器(编号, 设定值)
    
    // 比较指令
    Equal(u16, u16),        // 等于
    Greater(u16, u16),      // 大于
    Less(u16, u16),         // 小于
    
    // 数学指令
    Add(u16, u16, u16),     // 加法
    Sub(u16, u16, u16),     // 减法
    Mul(u16, u16, u16),     // 乘法
    Div(u16, u16, u16),     // 除法
}

/// 梯形图程序
pub struct LadderProgram {
    instructions: heapless::Vec<LadderInstruction, 1000>,
    memory: [bool; 1024],           // 位内存
    registers: [i32; 256],          // 寄存器
    timers: [Timer; 64],            // 定时器
    counters: [Counter; 64],        // 计数器
    edge_memory: [bool; 1024],      // 边沿检测内存
}

/// 定时器
#[derive(Clone, Copy)]
pub struct Timer {
    pub preset: u32,
    pub current: u32,
    pub enable: bool,
    pub done: bool,
}

/// 计数器
#[derive(Clone, Copy)]
pub struct Counter {
    pub preset: u32,
    pub current: u32,
    pub enable: bool,
    pub done: bool,
    pub reset: bool,
}

impl LadderProgram {
    pub fn new() -> Self {
        Self {
            instructions: heapless::Vec::new(),
            memory: [false; 1024],
            registers: [0; 256],
            timers: [Timer { preset: 0, current: 0, enable: false, done: false }; 64],
            counters: [Counter { preset: 0, current: 0, enable: false, done: false, reset: false }; 64],
            edge_memory: [false; 1024],
        }
    }
    
    /// 添加指令
    pub fn add_instruction(&mut self, instruction: LadderInstruction) -> Result<(), &'static str> {
        self.instructions.push(instruction).map_err(|_| "Program memory full")
    }
    
    /// 执行程序
    pub fn execute(&mut self) {
        let mut stack = heapless::Vec::<bool, 32>::new();
        let mut accumulator = false;
        
        for instruction in &self.instructions {
            match instruction {
                LadderInstruction::NormallyOpen(addr) => {
                    accumulator = self.memory[*addr as usize];
                }
                
                LadderInstruction::NormallyClosed(addr) => {
                    accumulator = !self.memory[*addr as usize];
                }
                
                LadderInstruction::RisingEdge(addr) => {
                    let current = self.memory[*addr as usize];
                    let previous = self.edge_memory[*addr as usize];
                    accumulator = current && !previous;
                    self.edge_memory[*addr as usize] = current;
                }
                
                LadderInstruction::FallingEdge(addr) => {
                    let current = self.memory[*addr as usize];
                    let previous = self.edge_memory[*addr as usize];
                    accumulator = !current && previous;
                    self.edge_memory[*addr as usize] = current;
                }
                
                LadderInstruction::Coil(addr) => {
                    self.memory[*addr as usize] = accumulator;
                }
                
                LadderInstruction::Set(addr) => {
                    if accumulator {
                        self.memory[*addr as usize] = true;
                    }
                }
                
                LadderInstruction::Reset(addr) => {
                    if accumulator {
                        self.memory[*addr as usize] = false;
                    }
                }
                
                LadderInstruction::And => {
                    if let Some(value) = stack.pop() {
                        accumulator = accumulator && value;
                    }
                }
                
                LadderInstruction::Or => {
                    if let Some(value) = stack.pop() {
                        accumulator = accumulator || value;
                    }
                }
                
                LadderInstruction::Not => {
                    accumulator = !accumulator;
                }
                
                LadderInstruction::Timer(id, preset) => {
                    let timer = &mut self.timers[*id as usize];
                    timer.enable = accumulator;
                    timer.preset = *preset;
                    
                    if timer.enable {
                        if timer.current < timer.preset {
                            timer.current += 1;
                        }
                        timer.done = timer.current >= timer.preset;
                    } else {
                        timer.current = 0;
                        timer.done = false;
                    }
                    
                    accumulator = timer.done;
                }
                
                LadderInstruction::Counter(id, preset) => {
                    let counter = &mut self.counters[*id as usize];
                    counter.preset = *preset;
                    
                    if accumulator && !counter.enable {
                        // 上升沿计数
                        counter.current += 1;
                    }
                    counter.enable = accumulator;
                    counter.done = counter.current >= counter.preset;
                    
                    if counter.reset {
                        counter.current = 0;
                        counter.done = false;
                    }
                    
                    accumulator = counter.done;
                }
                
                LadderInstruction::Equal(addr1, addr2) => {
                    accumulator = self.registers[*addr1 as usize] == self.registers[*addr2 as usize];
                }
                
                LadderInstruction::Greater(addr1, addr2) => {
                    accumulator = self.registers[*addr1 as usize] > self.registers[*addr2 as usize];
                }
                
                LadderInstruction::Less(addr1, addr2) => {
                    accumulator = self.registers[*addr1 as usize] < self.registers[*addr2 as usize];
                }
                
                LadderInstruction::Add(src1, src2, dest) => {
                    if accumulator {
                        self.registers[*dest as usize] = 
                            self.registers[*src1 as usize] + self.registers[*src2 as usize];
                    }
                }
                
                LadderInstruction::Sub(src1, src2, dest) => {
                    if accumulator {
                        self.registers[*dest as usize] = 
                            self.registers[*src1 as usize] - self.registers[*src2 as usize];
                    }
                }
                
                LadderInstruction::Mul(src1, src2, dest) => {
                    if accumulator {
                        self.registers[*dest as usize] = 
                            self.registers[*src1 as usize] * self.registers[*src2 as usize];
                    }
                }
                
                LadderInstruction::Div(src1, src2, dest) => {
                    if accumulator && self.registers[*src2 as usize] != 0 {
                        self.registers[*dest as usize] = 
                            self.registers[*src1 as usize] / self.registers[*src2 as usize];
                    }
                }
            }
        }
    }
    
    /// 设置位内存
    pub fn set_bit(&mut self, addr: u16, value: bool) {
        if (addr as usize) < self.memory.len() {
            self.memory[addr as usize] = value;
        }
    }
    
    /// 获取位内存
    pub fn get_bit(&self, addr: u16) -> bool {
        if (addr as usize) < self.memory.len() {
            self.memory[addr as usize]
        } else {
            false
        }
    }
    
    /// 设置寄存器
    pub fn set_register(&mut self, addr: u16, value: i32) {
        if (addr as usize) < self.registers.len() {
            self.registers[addr as usize] = value;
        }
    }
    
    /// 获取寄存器
    pub fn get_register(&self, addr: u16) -> i32 {
        if (addr as usize) < self.registers.len() {
            self.registers[addr as usize]
        } else {
            0
        }
    }
}
```

## 通信协议

### Modbus RTU实现
```rust
use heapless::Vec;

/// Modbus功能码
#[derive(Clone, Copy, PartialEq)]
pub enum ModbusFunction {
    ReadCoils = 0x01,
    ReadDiscreteInputs = 0x02,
    ReadHoldingRegisters = 0x03,
    ReadInputRegisters = 0x04,
    WriteSingleCoil = 0x05,
    WriteSingleRegister = 0x06,
    WriteMultipleCoils = 0x0F,
    WriteMultipleRegisters = 0x10,
}

/// Modbus异常码
#[derive(Clone, Copy, PartialEq)]
pub enum ModbusException {
    IllegalFunction = 0x01,
    IllegalDataAddress = 0x02,
    IllegalDataValue = 0x03,
    SlaveDeviceFailure = 0x04,
    Acknowledge = 0x05,
    SlaveDeviceBusy = 0x06,
}

/// Modbus RTU从站
pub struct ModbusRtuSlave {
    slave_id: u8,
    coils: [bool; 256],
    discrete_inputs: [bool; 256],
    holding_registers: [u16; 256],
    input_registers: [u16; 256],
}

impl ModbusRtuSlave {
    pub fn new(slave_id: u8) -> Self {
        Self {
            slave_id,
            coils: [false; 256],
            discrete_inputs: [false; 256],
            holding_registers: [0; 256],
            input_registers: [0; 256],
        }
    }
    
    /// 处理Modbus请求
    pub fn process_request(&mut self, request: &[u8]) -> Result<Vec<u8, 256>, ModbusException> {
        if request.len() < 4 {
            return Err(ModbusException::IllegalFunction);
        }
        
        let slave_id = request[0];
        if slave_id != self.slave_id && slave_id != 0 {
            return Err(ModbusException::IllegalFunction);
        }
        
        let function_code = request[1];
        let start_addr = ((request[2] as u16) << 8) | (request[3] as u16);
        
        match function_code {
            0x01 => self.read_coils(start_addr, request),
            0x02 => self.read_discrete_inputs(start_addr, request),
            0x03 => self.read_holding_registers(start_addr, request),
            0x04 => self.read_input_registers(start_addr, request),
            0x05 => self.write_single_coil(start_addr, request),
            0x06 => self.write_single_register(start_addr, request),
            0x0F => self.write_multiple_coils(start_addr, request),
            0x10 => self.write_multiple_registers(start_addr, request),
            _ => Err(ModbusException::IllegalFunction),
        }
    }
    
    /// 读取线圈
    fn read_coils(&self, start_addr: u16, request: &[u8]) -> Result<Vec<u8, 256>, ModbusException> {
        if request.len() < 6 {
            return Err(ModbusException::IllegalFunction);
        }
        
        let quantity = ((request[4] as u16) << 8) | (request[5] as u16);
        if quantity == 0 || quantity > 2000 {
            return Err(ModbusException::IllegalDataValue);
        }
        
        if start_addr as usize + quantity as usize > self.coils.len() {
            return Err(ModbusException::IllegalDataAddress);
        }
        
        let mut response = Vec::new();
        response.push(self.slave_id).ok();
        response.push(0x01).ok();
        
        let byte_count = (quantity + 7) / 8;
        response.push(byte_count as u8).ok();
        
        for byte_idx in 0..byte_count {
            let mut byte_value = 0u8;
            for bit_idx in 0..8 {
                let coil_idx = start_addr as usize + (byte_idx * 8 + bit_idx) as usize;
                if coil_idx < start_addr as usize + quantity as usize && coil_idx < self.coils.len() {
                    if self.coils[coil_idx] {
                        byte_value |= 1 << bit_idx;
                    }
                }
            }
            response.push(byte_value).ok();
        }
        
        Ok(response)
    }
    
    /// 读取保持寄存器
    fn read_holding_registers(&self, start_addr: u16, request: &[u8]) -> Result<Vec<u8, 256>, ModbusException> {
        if request.len() < 6 {
            return Err(ModbusException::IllegalFunction);
        }
        
        let quantity = ((request[4] as u16) << 8) | (request[5] as u16);
        if quantity == 0 || quantity > 125 {
            return Err(ModbusException::IllegalDataValue);
        }
        
        if start_addr as usize + quantity as usize > self.holding_registers.len() {
            return Err(ModbusException::IllegalDataAddress);
        }
        
        let mut response = Vec::new();
        response.push(self.slave_id).ok();
        response.push(0x03).ok();
        response.push((quantity * 2) as u8).ok();
        
        for i in 0..quantity {
            let reg_value = self.holding_registers[start_addr as usize + i as usize];
            response.push((reg_value >> 8) as u8).ok();
            response.push((reg_value & 0xFF) as u8).ok();
        }
        
        Ok(response)
    }
    
    /// 写单个线圈
    fn write_single_coil(&mut self, start_addr: u16, request: &[u8]) -> Result<Vec<u8, 256>, ModbusException> {
        if request.len() < 6 {
            return Err(ModbusException::IllegalFunction);
        }
        
        if start_addr as usize >= self.coils.len() {
            return Err(ModbusException::IllegalDataAddress);
        }
        
        let value = ((request[4] as u16) << 8) | (request[5] as u16);
        let coil_value = match value {
            0x0000 => false,
            0xFF00 => true,
            _ => return Err(ModbusException::IllegalDataValue),
        };
        
        self.coils[start_addr as usize] = coil_value;
        
        // 回显请求
        let mut response = Vec::new();
        for &byte in request.iter().take(6) {
            response.push(byte).ok();
        }
        
        Ok(response)
    }
    
    /// 写单个寄存器
    fn write_single_register(&mut self, start_addr: u16, request: &[u8]) -> Result<Vec<u8, 256>, ModbusException> {
        if request.len() < 6 {
            return Err(ModbusException::IllegalFunction);
        }
        
        if start_addr as usize >= self.holding_registers.len() {
            return Err(ModbusException::IllegalDataAddress);
        }
        
        let value = ((request[4] as u16) << 8) | (request[5] as u16);
        self.holding_registers[start_addr as usize] = value;
        
        // 回显请求
        let mut response = Vec::new();
        for &byte in request.iter().take(6) {
            response.push(byte).ok();
        }
        
        Ok(response)
    }
    
    // 其他功能码的实现...
    fn read_discrete_inputs(&self, _start_addr: u16, _request: &[u8]) -> Result<Vec<u8, 256>, ModbusException> {
        Err(ModbusException::IllegalFunction)
    }
    
    fn read_input_registers(&self, _start_addr: u16, _request: &[u8]) -> Result<Vec<u8, 256>, ModbusException> {
        Err(ModbusException::IllegalFunction)
    }
    
    fn write_multiple_coils(&mut self, _start_addr: u16, _request: &[u8]) -> Result<Vec<u8, 256>, ModbusException> {
        Err(ModbusException::IllegalFunction)
    }
    
    fn write_multiple_registers(&mut self, _start_addr: u16, _request: &[u8]) -> Result<Vec<u8, 256>, ModbusException> {
        Err(ModbusException::IllegalFunction)
    }
    
    /// 更新输入数据
    pub fn update_discrete_inputs(&mut self, inputs: &[bool]) {
        let len = inputs.len().min(self.discrete_inputs.len());
        self.discrete_inputs[..len].copy_from_slice(&inputs[..len]);
    }
    
    pub fn update_input_registers(&mut self, registers: &[u16]) {
        let len = registers.len().min(self.input_registers.len());
        self.input_registers[..len].copy_from_slice(&registers[..len]);
    }
    
    /// 获取输出数据
    pub fn get_coils(&self) -> &[bool; 256] {
        &self.coils
    }
    
    pub fn get_holding_registers(&self) -> &[u16; 256] {
        &self.holding_registers
    }
}

/// CRC16计算
pub fn calculate_crc16(data: &[u8]) -> u16 {
    let mut crc = 0xFFFFu16;
    
    for &byte in data {
        crc ^= byte as u16;
        for _ in 0..8 {
            if crc & 1 != 0 {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    
    crc
}
```

## 实时任务调度

### 任务调度器
```rust
use heapless::Vec;

/// 任务优先级
#[derive(Clone, Copy, PartialEq, PartialOrd)]
pub enum TaskPriority {
    Critical = 0,
    High = 1,
    Medium = 2,
    Low = 3,
}

/// 任务状态
#[derive(Clone, Copy, PartialEq)]
pub enum TaskState {
    Ready,
    Running,
    Blocked,
    Suspended,
}

/// 任务控制块
pub struct TaskControlBlock {
    pub id: u8,
    pub priority: TaskPriority,
    pub state: TaskState,
    pub period: u32,           // 任务周期 (ms)
    pub deadline: u32,         // 任务截止时间 (ms)
    pub execution_time: u32,   // 执行时间 (μs)
    pub last_run: u32,         // 上次运行时间
    pub next_run: u32,         // 下次运行时间
    pub run_count: u32,        // 运行次数
    pub overrun_count: u32,    // 超时次数
}

/// 实时任务调度器
pub struct RealTimeScheduler {
    tasks: Vec<TaskControlBlock, 16>,
    current_task: Option<u8>,
    system_tick: u32,
    scheduler_enabled: bool,
}

impl RealTimeScheduler {
    pub fn new() -> Self {
        Self {
            tasks: Vec::new(),
            current_task: None,
            system_tick: 0,
            scheduler_enabled: false,
        }
    }
    
    /// 添加任务
    pub fn add_task(&mut self, id: u8, priority: TaskPriority, period: u32) -> Result<(), &'static str> {
        let task = TaskControlBlock {
            id,
            priority,
            state: TaskState::Ready,
            period,
            deadline: period,
            execution_time: 0,
            last_run: 0,
            next_run: self.system_tick + period,
            run_count: 0,
            overrun_count: 0,
        };
        
        self.tasks.push(task).map_err(|_| "Task list full")
    }
    
    /// 启动调度器
    pub fn start(&mut self) {
        self.scheduler_enabled = true;
    }
    
    /// 停止调度器
    pub fn stop(&mut self) {
        self.scheduler_enabled = false;
    }
    
    /// 系统时钟中断处理
    pub fn tick(&mut self) {
        self.system_tick += 1;
        
        if !self.scheduler_enabled {
            return;
        }
        
        // 更新任务状态
        for task in &mut self.tasks {
            if task.state == TaskState::Ready && self.system_tick >= task.next_run {
                // 检查是否超过截止时间
                if self.system_tick > task.last_run + task.deadline {
                    task.overrun_count += 1;
                }
                
                // 设置下次运行时间
                task.next_run = self.system_tick + task.period;
            }
        }
    }
    
    /// 获取下一个要运行的任务
    pub fn get_next_task(&mut self) -> Option<u8> {
        if !self.scheduler_enabled {
            return None;
        }
        
        let mut next_task: Option<usize> = None;
        let mut highest_priority = TaskPriority::Low;
        let mut earliest_deadline = u32::MAX;
        
        // 优先级调度 + EDF (Earliest Deadline First)
        for (i, task) in self.tasks.iter().enumerate() {
            if task.state == TaskState::Ready && self.system_tick >= task.next_run {
                let deadline = task.last_run + task.deadline;
                
                if task.priority < highest_priority || 
                   (task.priority == highest_priority && deadline < earliest_deadline) {
                    next_task = Some(i);
                    highest_priority = task.priority;
                    earliest_deadline = deadline;
                }
            }
        }
        
        if let Some(task_idx) = next_task {
            let task_id = self.tasks[task_idx].id;
            self.tasks[task_idx].state = TaskState::Running;
            self.tasks[task_idx].last_run = self.system_tick;
            self.tasks[task_idx].run_count += 1;
            self.current_task = Some(task_id);
            return Some(task_id);
        }
        
        None
    }
    
    /// 任务完成
    pub fn task_complete(&mut self, task_id: u8, execution_time: u32) {
        for task in &mut self.tasks {
            if task.id == task_id {
                task.state = TaskState::Ready;
                task.execution_time = execution_time;
                break;
            }
        }
        self.current_task = None;
    }
    
    /// 获取任务统计信息
    pub fn get_task_stats(&self, task_id: u8) -> Option<&TaskControlBlock> {
        self.tasks.iter().find(|task| task.id == task_id)
    }
    
    /// 获取系统负载
    pub fn get_system_load(&self) -> f32 {
        let total_utilization: u32 = self.tasks.iter()
            .map(|task| (task.execution_time * 1000) / task.period)
            .sum();
        
        (total_utilization as f32) / 1000.0
    }
}
```

## 总结

本文档介绍了基于STM32F4的工业控制器基础知识，包括：

1. **I/O系统设计**: 数字输入/输出、模拟输入/输出模块的实现
2. **逻辑控制引擎**: 梯形图逻辑的解释执行
3. **通信协议**: Modbus RTU协议的实现
4. **实时任务调度**: 多任务实时调度系统

这些基础组件为构建完整的工业控制系统提供了坚实的基础。在实际应用中，还需要考虑系统的可靠性、安全性和可维护性等方面。