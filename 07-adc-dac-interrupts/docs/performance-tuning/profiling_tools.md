# STM32F4 ADC/DAC 系统性能分析工具指南

## 概述

性能分析是优化STM32F4 ADC/DAC系统的关键步骤。本文档详细介绍了各种性能分析工具、技术和方法，帮助开发者深入了解系统性能特征，识别瓶颈，并指导优化工作。

## 硬件性能分析工具

### 1. 内置性能监控单元 (PMU)

```rust
#[derive(Debug)]
pub struct PerformanceMonitoringUnit {
    pub dwt: DataWatchpointTrace,
    pub itm: InstrumentationTraceModule,
    pub tpiu: TracePortInterfaceUnit,
    pub etm: EmbeddedTraceModule,
    pub counters: PerformanceCounters,
}

#[derive(Debug)]
pub struct DataWatchpointTrace {
    pub cycle_counter: CycleCounter,
    pub comparators: Vec<DwtComparator>,
    pub exception_trace: ExceptionTrace,
    pub pc_sampling: PcSampling,
}

#[derive(Debug)]
pub struct CycleCounter {
    pub enabled: bool,
    pub count: u32,
    pub overflow_count: u32,
}

impl CycleCounter {
    pub fn enable(&mut self) {
        unsafe {
            // 启用DWT
            let dwt_ctrl = 0xE0001000 as *mut u32;
            core::ptr::write_volatile(dwt_ctrl, core::ptr::read_volatile(dwt_ctrl) | 1);
            
            // 启用CYCCNT
            let cyccnt_ena = 0xE0001000 as *mut u32;
            core::ptr::write_volatile(cyccnt_ena, core::ptr::read_volatile(cyccnt_ena) | (1 << 0));
            
            // 重置计数器
            let cyccnt = 0xE0001004 as *mut u32;
            core::ptr::write_volatile(cyccnt, 0);
        }
        self.enabled = true;
    }

    pub fn read(&self) -> u32 {
        if !self.enabled {
            return 0;
        }
        
        unsafe {
            core::ptr::read_volatile(0xE0001004 as *const u32)
        }
    }

    pub fn reset(&mut self) {
        unsafe {
            core::ptr::write_volatile(0xE0001004 as *mut u32, 0);
        }
    }

    pub fn get_elapsed_cycles(&self, start: u32) -> u32 {
        let current = self.read();
        if current >= start {
            current - start
        } else {
            // 处理溢出情况
            (0xFFFFFFFF - start) + current + 1
        }
    }

    pub fn cycles_to_microseconds(&self, cycles: u32, cpu_freq_hz: u32) -> f32 {
        (cycles as f32 / cpu_freq_hz as f32) * 1_000_000.0
    }
}

#[derive(Debug)]
pub struct DwtComparator {
    pub id: u8,
    pub address: u32,
    pub mask: u32,
    pub function: ComparatorFunction,
    pub enabled: bool,
    pub match_count: u32,
}

#[derive(Debug)]
pub enum ComparatorFunction {
    Disabled,
    InstructionAddress,
    DataAddress,
    DataValue,
    PcSample,
}

impl DwtComparator {
    pub fn configure_instruction_trace(&mut self, address: u32) {
        self.address = address;
        self.function = ComparatorFunction::InstructionAddress;
        self.enabled = true;
        
        unsafe {
            let comp_base = 0xE0001020 + (self.id as u32 * 16);
            
            // 设置地址
            core::ptr::write_volatile((comp_base + 0) as *mut u32, address);
            
            // 设置掩码
            core::ptr::write_volatile((comp_base + 4) as *mut u32, self.mask);
            
            // 设置功能
            core::ptr::write_volatile((comp_base + 8) as *mut u32, 4); // 指令地址匹配
        }
    }

    pub fn configure_data_watchpoint(&mut self, address: u32, size: WatchpointSize, access: WatchpointAccess) {
        self.address = address;
        self.function = ComparatorFunction::DataAddress;
        self.enabled = true;
        
        let function_value = match (size, access) {
            (WatchpointSize::Byte, WatchpointAccess::Read) => 5,
            (WatchpointSize::Byte, WatchpointAccess::Write) => 6,
            (WatchpointSize::Byte, WatchpointAccess::ReadWrite) => 7,
            (WatchpointSize::Halfword, WatchpointAccess::Read) => 9,
            (WatchpointSize::Halfword, WatchpointAccess::Write) => 10,
            (WatchpointSize::Halfword, WatchpointAccess::ReadWrite) => 11,
            (WatchpointSize::Word, WatchpointAccess::Read) => 13,
            (WatchpointSize::Word, WatchpointAccess::Write) => 14,
            (WatchpointSize::Word, WatchpointAccess::ReadWrite) => 15,
        };
        
        unsafe {
            let comp_base = 0xE0001020 + (self.id as u32 * 16);
            
            // 设置地址
            core::ptr::write_volatile((comp_base + 0) as *mut u32, address);
            
            // 设置掩码
            core::ptr::write_volatile((comp_base + 4) as *mut u32, 0);
            
            // 设置功能
            core::ptr::write_volatile((comp_base + 8) as *mut u32, function_value);
        }
    }
}

#[derive(Debug)]
pub enum WatchpointSize {
    Byte,
    Halfword,
    Word,
}

#[derive(Debug)]
pub enum WatchpointAccess {
    Read,
    Write,
    ReadWrite,
}
```

### 2. 性能计数器

```rust
#[derive(Debug)]
pub struct PerformanceCounters {
    pub instruction_count: InstructionCounter,
    pub cache_counters: CacheCounters,
    pub branch_counters: BranchCounters,
    pub memory_counters: MemoryCounters,
    pub interrupt_counters: InterruptCounters,
}

#[derive(Debug)]
pub struct InstructionCounter {
    pub total_instructions: u64,
    pub load_instructions: u32,
    pub store_instructions: u32,
    pub branch_instructions: u32,
    pub arithmetic_instructions: u32,
}

#[derive(Debug)]
pub struct CacheCounters {
    pub icache_hits: u32,
    pub icache_misses: u32,
    pub dcache_hits: u32,
    pub dcache_misses: u32,
    pub cache_hit_rate: f32,
}

#[derive(Debug)]
pub struct BranchCounters {
    pub total_branches: u32,
    pub taken_branches: u32,
    pub mispredicted_branches: u32,
    pub branch_prediction_rate: f32,
}

#[derive(Debug)]
pub struct MemoryCounters {
    pub memory_accesses: u32,
    pub memory_stalls: u32,
    pub dma_transfers: u32,
    pub memory_bandwidth_utilization: f32,
}

#[derive(Debug)]
pub struct InterruptCounters {
    pub total_interrupts: u32,
    pub adc_interrupts: u32,
    pub dac_interrupts: u32,
    pub timer_interrupts: u32,
    pub dma_interrupts: u32,
    pub interrupt_latency_histogram: Vec<u32>,
}

impl PerformanceCounters {
    pub fn new() -> Self {
        Self {
            instruction_count: InstructionCounter {
                total_instructions: 0,
                load_instructions: 0,
                store_instructions: 0,
                branch_instructions: 0,
                arithmetic_instructions: 0,
            },
            cache_counters: CacheCounters {
                icache_hits: 0,
                icache_misses: 0,
                dcache_hits: 0,
                dcache_misses: 0,
                cache_hit_rate: 0.0,
            },
            branch_counters: BranchCounters {
                total_branches: 0,
                taken_branches: 0,
                mispredicted_branches: 0,
                branch_prediction_rate: 0.0,
            },
            memory_counters: MemoryCounters {
                memory_accesses: 0,
                memory_stalls: 0,
                dma_transfers: 0,
                memory_bandwidth_utilization: 0.0,
            },
            interrupt_counters: InterruptCounters {
                total_interrupts: 0,
                adc_interrupts: 0,
                dac_interrupts: 0,
                timer_interrupts: 0,
                dma_interrupts: 0,
                interrupt_latency_histogram: vec![0; 100],
            },
        }
    }

    pub fn update_cache_statistics(&mut self) {
        let total_icache = self.cache_counters.icache_hits + self.cache_counters.icache_misses;
        if total_icache > 0 {
            self.cache_counters.cache_hit_rate = 
                self.cache_counters.icache_hits as f32 / total_icache as f32;
        }
    }

    pub fn update_branch_statistics(&mut self) {
        if self.branch_counters.total_branches > 0 {
            let correct_predictions = self.branch_counters.total_branches - 
                                    self.branch_counters.mispredicted_branches;
            self.branch_counters.branch_prediction_rate = 
                correct_predictions as f32 / self.branch_counters.total_branches as f32;
        }
    }

    pub fn record_interrupt_latency(&mut self, latency_us: u32) {
        self.interrupt_counters.total_interrupts += 1;
        
        // 将延迟记录到直方图中（以微秒为单位）
        let bucket = (latency_us / 10).min(99) as usize; // 每个桶代表10微秒
        self.interrupt_counters.interrupt_latency_histogram[bucket] += 1;
    }

    pub fn get_average_interrupt_latency(&self) -> f32 {
        let mut total_latency = 0.0;
        let mut total_count = 0;
        
        for (bucket, count) in self.interrupt_counters.interrupt_latency_histogram.iter().enumerate() {
            if *count > 0 {
                let bucket_center = (bucket * 10 + 5) as f32; // 桶的中心值
                total_latency += bucket_center * (*count as f32);
                total_count += count;
            }
        }
        
        if total_count > 0 {
            total_latency / total_count as f32
        } else {
            0.0
        }
    }
}
```

### 3. 跟踪和调试接口

```rust
#[derive(Debug)]
pub struct TraceInterface {
    pub swo: SerialWireOutput,
    pub etm: EmbeddedTraceModule,
    pub trace_buffer: TraceBuffer,
    pub trace_config: TraceConfig,
}

#[derive(Debug)]
pub struct SerialWireOutput {
    pub enabled: bool,
    pub baud_rate: u32,
    pub stimulus_ports: [bool; 32],
    pub timestamp_enabled: bool,
}

impl SerialWireOutput {
    pub fn initialize(&mut self, cpu_freq: u32, baud_rate: u32) {
        self.baud_rate = baud_rate;
        
        unsafe {
            // 配置TPIU
            let tpiu_base = 0xE0040000;
            
            // 设置当前端口大小为1（SWO）
            core::ptr::write_volatile((tpiu_base + 0x004) as *mut u32, 1);
            
            // 设置异步时钟预分频器
            let prescaler = (cpu_freq / baud_rate) - 1;
            core::ptr::write_volatile((tpiu_base + 0x010) as *mut u32, prescaler);
            
            // 选择SWO输出协议
            core::ptr::write_volatile((tpiu_base + 0x0F0) as *mut u32, 2); // NRZ模式
            
            // 启用ITM
            let itm_base = 0xE0000000;
            core::ptr::write_volatile((itm_base + 0xE00) as *mut u32, 0xC5ACCE55); // 解锁
            core::ptr::write_volatile((itm_base + 0xE80) as *mut u32, 1); // 启用ITM
            core::ptr::write_volatile((itm_base + 0xE40) as *mut u32, 1); // 启用时间戳
            
            // 启用刺激端口
            core::ptr::write_volatile((itm_base + 0xE00) as *mut u32, 0xFFFFFFFF);
        }
        
        self.enabled = true;
    }

    pub fn send_data(&self, port: u8, data: &[u8]) {
        if !self.enabled || port >= 32 || !self.stimulus_ports[port as usize] {
            return;
        }
        
        unsafe {
            let itm_base = 0xE0000000;
            let port_addr = itm_base + (port as u32 * 4);
            
            for &byte in data {
                // 等待端口就绪
                while (core::ptr::read_volatile(port_addr as *const u32) & 1) == 0 {}
                
                // 发送数据
                core::ptr::write_volatile(port_addr as *mut u8, byte);
            }
        }
    }

    pub fn send_string(&self, port: u8, s: &str) {
        self.send_data(port, s.as_bytes());
    }

    pub fn send_formatted(&self, port: u8, data: &dyn core::fmt::Debug) {
        // 简化实现，实际应用中需要格式化支持
        let formatted = format!("{:?}", data);
        self.send_string(port, &formatted);
    }
}

#[derive(Debug)]
pub struct TraceBuffer {
    pub buffer: Vec<TraceEvent>,
    pub capacity: usize,
    pub write_index: usize,
    pub overflow_count: u32,
}

#[derive(Debug, Clone)]
pub struct TraceEvent {
    pub timestamp: u32,
    pub event_type: TraceEventType,
    pub data: TraceEventData,
}

#[derive(Debug, Clone)]
pub enum TraceEventType {
    FunctionEntry,
    FunctionExit,
    InterruptEntry,
    InterruptExit,
    AdcConversion,
    DacUpdate,
    DmaTransfer,
    MemoryAccess,
    Custom,
}

#[derive(Debug, Clone)]
pub enum TraceEventData {
    FunctionCall {
        function_address: u32,
        parameters: Vec<u32>,
    },
    InterruptInfo {
        interrupt_number: u8,
        priority: u8,
        latency: u32,
    },
    AdcData {
        channel: u8,
        value: u16,
        conversion_time: u32,
    },
    DacData {
        channel: u8,
        value: u16,
        update_time: u32,
    },
    DmaInfo {
        channel: u8,
        source: u32,
        destination: u32,
        size: u32,
    },
    MemoryInfo {
        address: u32,
        size: u32,
        access_type: MemoryAccessType,
    },
    CustomData(Vec<u8>),
}

#[derive(Debug, Clone)]
pub enum MemoryAccessType {
    Read,
    Write,
    Execute,
}

impl TraceBuffer {
    pub fn new(capacity: usize) -> Self {
        Self {
            buffer: Vec::with_capacity(capacity),
            capacity,
            write_index: 0,
            overflow_count: 0,
        }
    }

    pub fn add_event(&mut self, event: TraceEvent) {
        if self.buffer.len() < self.capacity {
            self.buffer.push(event);
        } else {
            // 循环缓冲区
            self.buffer[self.write_index] = event;
            self.write_index = (self.write_index + 1) % self.capacity;
            self.overflow_count += 1;
        }
    }

    pub fn get_events(&self) -> &[TraceEvent] {
        &self.buffer
    }

    pub fn clear(&mut self) {
        self.buffer.clear();
        self.write_index = 0;
    }

    pub fn analyze_function_performance(&self) -> Vec<FunctionPerformance> {
        let mut function_stats = std::collections::HashMap::new();
        let mut call_stack = Vec::new();
        
        for event in &self.buffer {
            match &event.event_type {
                TraceEventType::FunctionEntry => {
                    if let TraceEventData::FunctionCall { function_address, .. } = &event.data {
                        call_stack.push((*function_address, event.timestamp));
                    }
                }
                TraceEventType::FunctionExit => {
                    if let Some((address, entry_time)) = call_stack.pop() {
                        let execution_time = event.timestamp - entry_time;
                        
                        let stats = function_stats.entry(address).or_insert(FunctionPerformance {
                            function_address: address,
                            call_count: 0,
                            total_time: 0,
                            min_time: u32::MAX,
                            max_time: 0,
                            average_time: 0.0,
                        });
                        
                        stats.call_count += 1;
                        stats.total_time += execution_time;
                        stats.min_time = stats.min_time.min(execution_time);
                        stats.max_time = stats.max_time.max(execution_time);
                        stats.average_time = stats.total_time as f32 / stats.call_count as f32;
                    }
                }
                _ => {}
            }
        }
        
        function_stats.into_values().collect()
    }
}

#[derive(Debug)]
pub struct FunctionPerformance {
    pub function_address: u32,
    pub call_count: u32,
    pub total_time: u32,
    pub min_time: u32,
    pub max_time: u32,
    pub average_time: f32,
}
```

## 软件性能分析工具

### 1. 性能分析器

```rust
#[derive(Debug)]
pub struct Profiler {
    pub sampling_profiler: SamplingProfiler,
    pub instrumentation_profiler: InstrumentationProfiler,
    pub memory_profiler: MemoryProfiler,
    pub call_graph_profiler: CallGraphProfiler,
}

#[derive(Debug)]
pub struct SamplingProfiler {
    pub enabled: bool,
    pub sample_interval: u32,
    pub samples: Vec<ProfileSample>,
    pub total_samples: u32,
    pub timer_config: TimerConfig,
}

#[derive(Debug)]
pub struct ProfileSample {
    pub timestamp: u32,
    pub program_counter: u32,
    pub stack_pointer: u32,
    pub cpu_state: CpuState,
    pub interrupt_context: bool,
}

#[derive(Debug)]
pub struct CpuState {
    pub registers: [u32; 16],
    pub psr: u32,
    pub control: u32,
}

impl SamplingProfiler {
    pub fn start(&mut self, sample_rate_hz: u32) {
        self.sample_interval = 1_000_000 / sample_rate_hz; // 微秒
        self.samples.clear();
        self.total_samples = 0;
        
        // 配置定时器进行采样
        self.configure_sampling_timer(sample_rate_hz);
        self.enabled = true;
    }

    pub fn stop(&mut self) {
        self.enabled = false;
        self.disable_sampling_timer();
    }

    pub fn take_sample(&mut self) {
        if !self.enabled {
            return;
        }
        
        let sample = ProfileSample {
            timestamp: self.get_timestamp(),
            program_counter: self.read_program_counter(),
            stack_pointer: self.read_stack_pointer(),
            cpu_state: self.capture_cpu_state(),
            interrupt_context: self.is_in_interrupt(),
        };
        
        self.samples.push(sample);
        self.total_samples += 1;
    }

    pub fn analyze_hotspots(&self) -> Vec<Hotspot> {
        let mut pc_counts = std::collections::HashMap::new();
        
        for sample in &self.samples {
            *pc_counts.entry(sample.program_counter).or_insert(0) += 1;
        }
        
        let mut hotspots: Vec<Hotspot> = pc_counts
            .into_iter()
            .map(|(pc, count)| Hotspot {
                program_counter: pc,
                sample_count: count,
                percentage: (count as f32 / self.total_samples as f32) * 100.0,
                function_name: self.resolve_function_name(pc),
            })
            .collect();
        
        hotspots.sort_by(|a, b| b.sample_count.cmp(&a.sample_count));
        hotspots
    }

    fn configure_sampling_timer(&mut self, sample_rate_hz: u32) {
        // 配置SysTick或其他定时器进行定期采样
        unsafe {
            let systick_base = 0xE000E010;
            
            // 计算重载值
            let reload_value = (168_000_000 / sample_rate_hz) - 1;
            
            // 配置SysTick
            core::ptr::write_volatile((systick_base + 0x04) as *mut u32, reload_value);
            core::ptr::write_volatile((systick_base + 0x08) as *mut u32, 0);
            core::ptr::write_volatile((systick_base + 0x00) as *mut u32, 0x07); // 启用，中断，时钟源
        }
    }

    fn read_program_counter(&self) -> u32 {
        // 读取程序计数器
        unsafe {
            let mut pc: u32;
            core::asm!("mov {}, pc", out(reg) pc);
            pc
        }
    }

    fn read_stack_pointer(&self) -> u32 {
        // 读取栈指针
        unsafe {
            let mut sp: u32;
            core::asm!("mov {}, sp", out(reg) sp);
            sp
        }
    }

    fn resolve_function_name(&self, pc: u32) -> Option<String> {
        // 解析函数名（需要符号表支持）
        None // 简化实现
    }
}

#[derive(Debug)]
pub struct Hotspot {
    pub program_counter: u32,
    pub sample_count: u32,
    pub percentage: f32,
    pub function_name: Option<String>,
}

#[derive(Debug)]
pub struct InstrumentationProfiler {
    pub function_entries: Vec<FunctionEntry>,
    pub call_stack: Vec<CallFrame>,
    pub overhead_compensation: bool,
}

#[derive(Debug)]
pub struct FunctionEntry {
    pub function_id: u32,
    pub entry_time: u32,
    pub exit_time: Option<u32>,
    pub call_depth: u32,
    pub parameters: Vec<u32>,
}

#[derive(Debug)]
pub struct CallFrame {
    pub function_id: u32,
    pub entry_time: u32,
    pub caller_id: Option<u32>,
}

impl InstrumentationProfiler {
    pub fn function_enter(&mut self, function_id: u32, parameters: Vec<u32>) {
        let entry_time = self.get_high_precision_time();
        
        let entry = FunctionEntry {
            function_id,
            entry_time,
            exit_time: None,
            call_depth: self.call_stack.len() as u32,
            parameters,
        };
        
        let frame = CallFrame {
            function_id,
            entry_time,
            caller_id: self.call_stack.last().map(|f| f.function_id),
        };
        
        self.function_entries.push(entry);
        self.call_stack.push(frame);
    }

    pub fn function_exit(&mut self, function_id: u32) -> Option<u32> {
        let exit_time = self.get_high_precision_time();
        
        if let Some(frame) = self.call_stack.pop() {
            if frame.function_id == function_id {
                // 更新函数条目
                if let Some(entry) = self.function_entries
                    .iter_mut()
                    .rev()
                    .find(|e| e.function_id == function_id && e.exit_time.is_none()) {
                    entry.exit_time = Some(exit_time);
                    return Some(exit_time - entry.entry_time);
                }
            }
        }
        
        None
    }

    pub fn generate_call_graph(&self) -> CallGraph {
        let mut nodes = std::collections::HashMap::new();
        let mut edges = std::collections::HashMap::new();
        
        for entry in &self.function_entries {
            // 添加节点
            let node = nodes.entry(entry.function_id).or_insert(CallGraphNode {
                function_id: entry.function_id,
                call_count: 0,
                total_time: 0,
                self_time: 0,
                children: Vec::new(),
            });
            
            node.call_count += 1;
            
            if let Some(exit_time) = entry.exit_time {
                node.total_time += exit_time - entry.entry_time;
            }
            
            // 添加边（调用关系）
            if let Some(caller_frame) = self.call_stack.iter()
                .find(|f| f.entry_time <= entry.entry_time) {
                let edge_key = (caller_frame.function_id, entry.function_id);
                *edges.entry(edge_key).or_insert(0) += 1;
            }
        }
        
        CallGraph {
            nodes: nodes.into_values().collect(),
            edges: edges.into_iter().map(|((caller, callee), count)| CallGraphEdge {
                caller_id: caller,
                callee_id: callee,
                call_count: count,
            }).collect(),
        }
    }
}

#[derive(Debug)]
pub struct CallGraph {
    pub nodes: Vec<CallGraphNode>,
    pub edges: Vec<CallGraphEdge>,
}

#[derive(Debug)]
pub struct CallGraphNode {
    pub function_id: u32,
    pub call_count: u32,
    pub total_time: u32,
    pub self_time: u32,
    pub children: Vec<u32>,
}

#[derive(Debug)]
pub struct CallGraphEdge {
    pub caller_id: u32,
    pub callee_id: u32,
    pub call_count: u32,
}
```

### 2. 内存分析器

```rust
#[derive(Debug)]
pub struct MemoryProfiler {
    pub heap_tracker: HeapTracker,
    pub stack_analyzer: StackAnalyzer,
    pub allocation_tracker: AllocationTracker,
    pub fragmentation_analyzer: FragmentationAnalyzer,
}

#[derive(Debug)]
pub struct HeapTracker {
    pub allocations: Vec<AllocationRecord>,
    pub total_allocated: usize,
    pub total_freed: usize,
    pub peak_usage: usize,
    pub current_usage: usize,
    pub allocation_count: u32,
    pub free_count: u32,
}

#[derive(Debug)]
pub struct AllocationRecord {
    pub address: usize,
    pub size: usize,
    pub timestamp: u32,
    pub call_stack: Vec<u32>,
    pub freed: bool,
    pub free_timestamp: Option<u32>,
}

impl HeapTracker {
    pub fn record_allocation(&mut self, address: usize, size: usize, call_stack: Vec<u32>) {
        let record = AllocationRecord {
            address,
            size,
            timestamp: self.get_timestamp(),
            call_stack,
            freed: false,
            free_timestamp: None,
        };
        
        self.allocations.push(record);
        self.total_allocated += size;
        self.current_usage += size;
        self.allocation_count += 1;
        
        if self.current_usage > self.peak_usage {
            self.peak_usage = self.current_usage;
        }
    }

    pub fn record_deallocation(&mut self, address: usize) {
        if let Some(record) = self.allocations.iter_mut()
            .find(|r| r.address == address && !r.freed) {
            record.freed = true;
            record.free_timestamp = Some(self.get_timestamp());
            
            self.total_freed += record.size;
            self.current_usage -= record.size;
            self.free_count += 1;
        }
    }

    pub fn detect_memory_leaks(&self) -> Vec<MemoryLeak> {
        let mut leaks = Vec::new();
        
        for record in &self.allocations {
            if !record.freed {
                let age = self.get_timestamp() - record.timestamp;
                if age > 10000 { // 10秒未释放视为潜在泄漏
                    leaks.push(MemoryLeak {
                        address: record.address,
                        size: record.size,
                        age,
                        allocation_stack: record.call_stack.clone(),
                    });
                }
            }
        }
        
        leaks.sort_by(|a, b| b.size.cmp(&a.size));
        leaks
    }

    pub fn analyze_allocation_patterns(&self) -> AllocationPattern {
        let mut size_histogram = vec![0u32; 20]; // 20个大小桶
        let mut lifetime_histogram = vec![0u32; 10]; // 10个生命周期桶
        
        for record in &self.allocations {
            // 大小分析
            let size_bucket = (record.size.ilog2() as usize).min(19);
            size_histogram[size_bucket] += 1;
            
            // 生命周期分析
            if let Some(free_time) = record.free_timestamp {
                let lifetime = free_time - record.timestamp;
                let lifetime_bucket = (lifetime / 1000).min(9) as usize; // 每桶1秒
                lifetime_histogram[lifetime_bucket] += 1;
            }
        }
        
        AllocationPattern {
            size_distribution: size_histogram,
            lifetime_distribution: lifetime_histogram,
            average_allocation_size: self.total_allocated / self.allocation_count.max(1) as usize,
            allocation_frequency: self.allocation_count as f32 / self.get_uptime() as f32,
        }
    }

    fn get_timestamp(&self) -> u32 {
        // 获取时间戳
        0 // 简化实现
    }

    fn get_uptime(&self) -> u32 {
        // 获取系统运行时间
        10000 // 简化实现
    }
}

#[derive(Debug)]
pub struct MemoryLeak {
    pub address: usize,
    pub size: usize,
    pub age: u32,
    pub allocation_stack: Vec<u32>,
}

#[derive(Debug)]
pub struct AllocationPattern {
    pub size_distribution: Vec<u32>,
    pub lifetime_distribution: Vec<u32>,
    pub average_allocation_size: usize,
    pub allocation_frequency: f32,
}

#[derive(Debug)]
pub struct StackAnalyzer {
    pub stack_base: usize,
    pub stack_size: usize,
    pub max_usage: usize,
    pub current_usage: usize,
    pub usage_history: Vec<StackUsageRecord>,
}

#[derive(Debug)]
pub struct StackUsageRecord {
    pub timestamp: u32,
    pub usage: usize,
    pub function_context: Option<u32>,
}

impl StackAnalyzer {
    pub fn measure_stack_usage(&mut self) {
        let current_sp = self.get_stack_pointer();
        let usage = self.stack_base - current_sp;
        
        self.current_usage = usage;
        if usage > self.max_usage {
            self.max_usage = usage;
        }
        
        self.usage_history.push(StackUsageRecord {
            timestamp: self.get_timestamp(),
            usage,
            function_context: self.get_current_function(),
        });
        
        // 保持历史记录大小
        if self.usage_history.len() > 1000 {
            self.usage_history.remove(0);
        }
    }

    pub fn analyze_stack_growth(&self) -> StackGrowthAnalysis {
        if self.usage_history.len() < 2 {
            return StackGrowthAnalysis::default();
        }
        
        let mut growth_events = Vec::new();
        let mut max_growth_rate = 0.0;
        
        for window in self.usage_history.windows(2) {
            let prev = &window[0];
            let curr = &window[1];
            
            if curr.usage > prev.usage {
                let growth = curr.usage - prev.usage;
                let time_diff = curr.timestamp - prev.timestamp;
                let growth_rate = growth as f32 / time_diff as f32;
                
                if growth_rate > max_growth_rate {
                    max_growth_rate = growth_rate;
                }
                
                growth_events.push(StackGrowthEvent {
                    timestamp: curr.timestamp,
                    growth_bytes: growth,
                    growth_rate,
                    function_context: curr.function_context,
                });
            }
        }
        
        StackGrowthAnalysis {
            max_usage: self.max_usage,
            average_usage: self.usage_history.iter().map(|r| r.usage).sum::<usize>() / self.usage_history.len(),
            growth_events,
            max_growth_rate,
            stack_utilization: (self.max_usage as f32 / self.stack_size as f32) * 100.0,
        }
    }

    fn get_stack_pointer(&self) -> usize {
        unsafe {
            let mut sp: usize;
            core::asm!("mov {}, sp", out(reg) sp);
            sp
        }
    }

    fn get_timestamp(&self) -> u32 {
        0 // 简化实现
    }

    fn get_current_function(&self) -> Option<u32> {
        None // 简化实现
    }
}

#[derive(Debug, Default)]
pub struct StackGrowthAnalysis {
    pub max_usage: usize,
    pub average_usage: usize,
    pub growth_events: Vec<StackGrowthEvent>,
    pub max_growth_rate: f32,
    pub stack_utilization: f32,
}

#[derive(Debug)]
pub struct StackGrowthEvent {
    pub timestamp: u32,
    pub growth_bytes: usize,
    pub growth_rate: f32,
    pub function_context: Option<u32>,
}
```

## 实时性能监控

### 1. 实时监控系统

```rust
#[derive(Debug)]
pub struct RealTimeMonitor {
    pub performance_dashboard: PerformanceDashboard,
    pub alert_system: AlertSystem,
    pub data_logger: DataLogger,
    pub visualization_engine: VisualizationEngine,
}

#[derive(Debug)]
pub struct PerformanceDashboard {
    pub widgets: Vec<DashboardWidget>,
    pub update_interval: u32,
    pub auto_refresh: bool,
    pub layout: DashboardLayout,
}

#[derive(Debug)]
pub enum DashboardWidget {
    CpuUsageGauge {
        current_value: f32,
        threshold: f32,
        history: Vec<f32>,
    },
    MemoryUsageChart {
        heap_usage: f32,
        stack_usage: f32,
        fragmentation: f32,
    },
    InterruptLatencyHistogram {
        bins: Vec<u32>,
        max_latency: f32,
        average_latency: f32,
    },
    ThroughputMeter {
        adc_throughput: f32,
        dac_throughput: f32,
        target_throughput: f32,
    },
    SystemHealthIndicator {
        overall_health: f32,
        component_status: Vec<ComponentStatus>,
    },
    PerformanceTimeline {
        events: Vec<PerformanceEvent>,
        time_window: u32,
    },
}

#[derive(Debug)]
pub struct ComponentStatus {
    pub component_name: String,
    pub status: HealthStatus,
    pub last_update: u32,
    pub metrics: Vec<f32>,
}

#[derive(Debug)]
pub enum HealthStatus {
    Healthy,
    Warning,
    Critical,
    Unknown,
}

#[derive(Debug)]
pub struct PerformanceEvent {
    pub timestamp: u32,
    pub event_type: EventType,
    pub severity: EventSeverity,
    pub description: String,
    pub metrics: Vec<f32>,
}

#[derive(Debug)]
pub enum EventType {
    PerformanceThresholdExceeded,
    SystemBottleneckDetected,
    MemoryLeakDetected,
    InterruptStormDetected,
    ThermalThrottling,
    PowerAnomalyDetected,
}

#[derive(Debug)]
pub enum EventSeverity {
    Info,
    Warning,
    Error,
    Critical,
}

impl RealTimeMonitor {
    pub fn update_dashboard(&mut self) {
        for widget in &mut self.performance_dashboard.widgets {
            match widget {
                DashboardWidget::CpuUsageGauge { current_value, history, .. } => {
                    *current_value = self.measure_cpu_usage();
                    history.push(*current_value);
                    if history.len() > 100 {
                        history.remove(0);
                    }
                }
                DashboardWidget::MemoryUsageChart { heap_usage, stack_usage, fragmentation } => {
                    let memory_stats = self.measure_memory_usage();
                    *heap_usage = memory_stats.heap_utilization;
                    *stack_usage = memory_stats.stack_utilization;
                    *fragmentation = memory_stats.fragmentation;
                }
                DashboardWidget::InterruptLatencyHistogram { bins, max_latency, average_latency } => {
                    let latency_stats = self.measure_interrupt_latency();
                    *bins = latency_stats.histogram;
                    *max_latency = latency_stats.max_latency;
                    *average_latency = latency_stats.average_latency;
                }
                DashboardWidget::ThroughputMeter { adc_throughput, dac_throughput, .. } => {
                    let throughput_stats = self.measure_throughput();
                    *adc_throughput = throughput_stats.adc_samples_per_second;
                    *dac_throughput = throughput_stats.dac_updates_per_second;
                }
                DashboardWidget::SystemHealthIndicator { overall_health, component_status } => {
                    *overall_health = self.calculate_system_health();
                    *component_status = self.get_component_status();
                }
                DashboardWidget::PerformanceTimeline { events, time_window } => {
                    let current_time = self.get_timestamp();
                    events.retain(|e| current_time - e.timestamp < *time_window);
                    events.extend(self.get_recent_events());
                }
            }
        }
    }

    pub fn detect_anomalies(&mut self) -> Vec<PerformanceAnomaly> {
        let mut anomalies = Vec::new();
        
        // CPU使用率异常检测
        let cpu_usage = self.measure_cpu_usage();
        if cpu_usage > 90.0 {
            anomalies.push(PerformanceAnomaly {
                anomaly_type: AnomalyType::HighCpuUsage,
                severity: AnomalySeverity::High,
                value: cpu_usage,
                threshold: 90.0,
                timestamp: self.get_timestamp(),
                description: format!("CPU使用率过高: {:.1}%", cpu_usage),
            });
        }
        
        // 内存使用异常检测
        let memory_stats = self.measure_memory_usage();
        if memory_stats.heap_utilization > 85.0 {
            anomalies.push(PerformanceAnomaly {
                anomaly_type: AnomalyType::HighMemoryUsage,
                severity: AnomalySeverity::Medium,
                value: memory_stats.heap_utilization,
                threshold: 85.0,
                timestamp: self.get_timestamp(),
                description: format!("堆内存使用率过高: {:.1}%", memory_stats.heap_utilization),
            });
        }
        
        // 中断延迟异常检测
        let latency_stats = self.measure_interrupt_latency();
        if latency_stats.max_latency > 100.0 {
            anomalies.push(PerformanceAnomaly {
                anomaly_type: AnomalyType::HighInterruptLatency,
                severity: AnomalySeverity::Critical,
                value: latency_stats.max_latency,
                threshold: 100.0,
                timestamp: self.get_timestamp(),
                description: format!("中断延迟过高: {:.1}μs", latency_stats.max_latency),
            });
        }
        
        anomalies
    }

    fn measure_cpu_usage(&self) -> f32 {
        // 测量CPU使用率
        75.0 // 简化实现
    }

    fn measure_memory_usage(&self) -> MemoryStats {
        // 测量内存使用情况
        MemoryStats {
            heap_utilization: 60.0,
            stack_utilization: 40.0,
            fragmentation: 15.0,
        }
    }

    fn measure_interrupt_latency(&self) -> InterruptLatencyStats {
        // 测量中断延迟
        InterruptLatencyStats {
            histogram: vec![10, 20, 30, 25, 15, 5, 2, 1, 0, 0],
            max_latency: 85.0,
            average_latency: 25.0,
        }
    }

    fn measure_throughput(&self) -> ThroughputStats {
        // 测量吞吐量
        ThroughputStats {
            adc_samples_per_second: 1_500_000.0,
            dac_updates_per_second: 800_000.0,
        }
    }

    fn calculate_system_health(&self) -> f32 {
        // 计算系统健康度
        85.0 // 简化实现
    }

    fn get_component_status(&self) -> Vec<ComponentStatus> {
        // 获取组件状态
        vec![
            ComponentStatus {
                component_name: "ADC".to_string(),
                status: HealthStatus::Healthy,
                last_update: self.get_timestamp(),
                metrics: vec![95.0, 2.5, 1500000.0],
            },
            ComponentStatus {
                component_name: "DAC".to_string(),
                status: HealthStatus::Warning,
                last_update: self.get_timestamp(),
                metrics: vec![88.0, 5.2, 800000.0],
            },
        ]
    }

    fn get_recent_events(&self) -> Vec<PerformanceEvent> {
        // 获取最近的性能事件
        Vec::new() // 简化实现
    }

    fn get_timestamp(&self) -> u32 {
        // 获取时间戳
        0 // 简化实现
    }
}

#[derive(Debug)]
pub struct MemoryStats {
    pub heap_utilization: f32,
    pub stack_utilization: f32,
    pub fragmentation: f32,
}

#[derive(Debug)]
pub struct InterruptLatencyStats {
    pub histogram: Vec<u32>,
    pub max_latency: f32,
    pub average_latency: f32,
}

#[derive(Debug)]
pub struct ThroughputStats {
    pub adc_samples_per_second: f32,
    pub dac_updates_per_second: f32,
}

#[derive(Debug)]
pub struct PerformanceAnomaly {
    pub anomaly_type: AnomalyType,
    pub severity: AnomalySeverity,
    pub value: f32,
    pub threshold: f32,
    pub timestamp: u32,
    pub description: String,
}

#[derive(Debug)]
pub enum AnomalyType {
    HighCpuUsage,
    HighMemoryUsage,
    HighInterruptLatency,
    LowThroughput,
    MemoryLeak,
    ThermalThrottling,
}

#[derive(Debug)]
pub enum AnomalySeverity {
    Low,
    Medium,
    High,
    Critical,
}
```

## 外部工具集成

### 1. 调试器集成

```rust
#[derive(Debug)]
pub struct DebuggerInterface {
    pub gdb_interface: GdbInterface,
    pub openocd_interface: OpenOcdInterface,
    pub trace_analyzer: TraceAnalyzer,
    pub symbol_resolver: SymbolResolver,
}

#[derive(Debug)]
pub struct GdbInterface {
    pub connection_status: ConnectionStatus,
    pub breakpoints: Vec<Breakpoint>,
    pub watchpoints: Vec<Watchpoint>,
    pub performance_commands: Vec<GdbCommand>,
}

#[derive(Debug)]
pub enum ConnectionStatus {
    Connected,
    Disconnected,
    Error(String),
}

#[derive(Debug)]
pub struct Breakpoint {
    pub id: u32,
    pub address: u32,
    pub condition: Option<String>,
    pub hit_count: u32,
    pub enabled: bool,
}

#[derive(Debug)]
pub struct Watchpoint {
    pub id: u32,
    pub address: u32,
    pub size: u32,
    pub access_type: WatchpointAccess,
    pub condition: Option<String>,
    pub hit_count: u32,
}

#[derive(Debug)]
pub struct GdbCommand {
    pub name: String,
    pub description: String,
    pub command: String,
    pub output_parser: fn(&str) -> Option<PerformanceData>,
}

impl GdbInterface {
    pub fn setup_performance_breakpoints(&mut self) -> Result<(), DebugError> {
        // 在关键函数设置性能测量断点
        let critical_functions = vec![
            ("adc_conversion_complete", 0x08001000),
            ("dac_update_complete", 0x08001100),
            ("dma_transfer_complete", 0x08001200),
        ];
        
        for (name, address) in critical_functions {
            let breakpoint = Breakpoint {
                id: self.get_next_breakpoint_id(),
                address,
                condition: None,
                hit_count: 0,
                enabled: true,
            };
            
            self.breakpoints.push(breakpoint);
        }
        
        Ok(())
    }

    pub fn collect_performance_data(&mut self) -> Result<Vec<PerformanceData>, DebugError> {
        let mut data = Vec::new();
        
        for command in &self.performance_commands {
            let output = self.execute_gdb_command(&command.command)?;
            if let Some(perf_data) = (command.output_parser)(&output) {
                data.push(perf_data);
            }
        }
        
        Ok(data)
    }

    fn get_next_breakpoint_id(&self) -> u32 {
        self.breakpoints.len() as u32 + 1
    }

    fn execute_gdb_command(&self, command: &str) -> Result<String, DebugError> {
        // 执行GDB命令
        Ok(String::new()) // 简化实现
    }
}

#[derive(Debug)]
pub enum DebugError {
    ConnectionError,
    CommandError(String),
    ParseError(String),
}

#[derive(Debug)]
pub struct PerformanceData {
    pub timestamp: u32,
    pub data_type: PerformanceDataType,
    pub value: f32,
    pub context: String,
}

#[derive(Debug)]
pub enum PerformanceDataType {
    ExecutionTime,
    MemoryUsage,
    RegisterValue,
    StackDepth,
    InterruptLatency,
}
```

### 2. 静态分析工具

```rust
#[derive(Debug)]
pub struct StaticAnalyzer {
    pub code_analyzer: CodeAnalyzer,
    pub complexity_analyzer: ComplexityAnalyzer,
    pub dependency_analyzer: DependencyAnalyzer,
    pub optimization_analyzer: OptimizationAnalyzer,
}

#[derive(Debug)]
pub struct CodeAnalyzer {
    pub metrics: CodeMetrics,
    pub hotspot_predictions: Vec<HotspotPrediction>,
    pub performance_warnings: Vec<PerformanceWarning>,
}

#[derive(Debug)]
pub struct CodeMetrics {
    pub lines_of_code: u32,
    pub cyclomatic_complexity: u32,
    pub function_count: u32,
    pub average_function_size: f32,
    pub max_function_size: u32,
    pub nesting_depth: u32,
}

#[derive(Debug)]
pub struct HotspotPrediction {
    pub function_name: String,
    pub predicted_cpu_usage: f32,
    pub confidence: f32,
    pub reasoning: String,
}

#[derive(Debug)]
pub struct PerformanceWarning {
    pub warning_type: WarningType,
    pub severity: WarningSeverity,
    pub location: SourceLocation,
    pub description: String,
    pub suggestion: String,
}

#[derive(Debug)]
pub enum WarningType {
    InefficiientLoop,
    UnoptimizedMemoryAccess,
    ExcessiveRecursion,
    LargeStackUsage,
    FrequentAllocation,
    UnalignedAccess,
    MissedOptimization,
}

#[derive(Debug)]
pub enum WarningSeverity {
    Info,
    Minor,
    Major,
    Critical,
}

#[derive(Debug)]
pub struct SourceLocation {
    pub file: String,
    pub line: u32,
    pub column: u32,
    pub function: String,
}

impl StaticAnalyzer {
    pub fn analyze_performance_characteristics(&mut self, source_files: &[String]) -> AnalysisReport {
        let mut report = AnalysisReport::new();
        
        for file in source_files {
            let file_analysis = self.analyze_file(file);
            report.file_reports.push(file_analysis);
        }
        
        report.overall_metrics = self.calculate_overall_metrics(&report.file_reports);
        report.recommendations = self.generate_recommendations(&report);
        
        report
    }

    fn analyze_file(&mut self, file_path: &str) -> FileAnalysisReport {
        // 分析单个文件
        FileAnalysisReport {
            file_path: file_path.to_string(),
            metrics: self.calculate_file_metrics(file_path),
            functions: self.analyze_functions(file_path),
            performance_issues: self.detect_performance_issues(file_path),
        }
    }

    fn calculate_file_metrics(&self, file_path: &str) -> CodeMetrics {
        // 计算文件代码指标
        CodeMetrics {
            lines_of_code: 1000,
            cyclomatic_complexity: 25,
            function_count: 20,
            average_function_size: 50.0,
            max_function_size: 200,
            nesting_depth: 4,
        }
    }

    fn analyze_functions(&self, file_path: &str) -> Vec<FunctionAnalysis> {
        // 分析函数性能特征
        vec![
            FunctionAnalysis {
                name: "adc_read_channel".to_string(),
                complexity: 8,
                estimated_cycles: 150,
                memory_usage: 64,
                optimization_level: OptimizationLevel::Good,
                performance_score: 85.0,
            }
        ]
    }

    fn detect_performance_issues(&self, file_path: &str) -> Vec<PerformanceIssue> {
        // 检测性能问题
        vec![
            PerformanceIssue {
                issue_type: IssueType::InefficiientLoop,
                severity: IssueSeverity::Medium,
                location: SourceLocation {
                    file: file_path.to_string(),
                    line: 125,
                    column: 8,
                    function: "process_samples".to_string(),
                },
                description: "循环中存在不必要的重复计算".to_string(),
                impact: "可能导致CPU使用率增加15%".to_string(),
                suggestion: "将重复计算移出循环".to_string(),
            }
        ]
    }
}

#[derive(Debug)]
pub struct AnalysisReport {
    pub file_reports: Vec<FileAnalysisReport>,
    pub overall_metrics: CodeMetrics,
    pub recommendations: Vec<OptimizationRecommendation>,
}

impl AnalysisReport {
    pub fn new() -> Self {
        Self {
            file_reports: Vec::new(),
            overall_metrics: CodeMetrics {
                lines_of_code: 0,
                cyclomatic_complexity: 0,
                function_count: 0,
                average_function_size: 0.0,
                max_function_size: 0,
                nesting_depth: 0,
            },
            recommendations: Vec::new(),
        }
    }
}

#[derive(Debug)]
pub struct FileAnalysisReport {
    pub file_path: String,
    pub metrics: CodeMetrics,
    pub functions: Vec<FunctionAnalysis>,
    pub performance_issues: Vec<PerformanceIssue>,
}

#[derive(Debug)]
pub struct FunctionAnalysis {
    pub name: String,
    pub complexity: u32,
    pub estimated_cycles: u32,
    pub memory_usage: usize,
    pub optimization_level: OptimizationLevel,
    pub performance_score: f32,
}

#[derive(Debug)]
pub enum OptimizationLevel {
    Poor,
    Fair,
    Good,
    Excellent,
}

#[derive(Debug)]
pub struct PerformanceIssue {
    pub issue_type: IssueType,
    pub severity: IssueSeverity,
    pub location: SourceLocation,
    pub description: String,
    pub impact: String,
    pub suggestion: String,
}

#[derive(Debug)]
pub enum IssueType {
    InefficiientLoop,
    UnoptimizedMemoryAccess,
    ExcessiveRecursion,
    LargeStackUsage,
    FrequentAllocation,
}

#[derive(Debug)]
pub enum IssueSeverity {
    Low,
    Medium,
    High,
    Critical,
}

#[derive(Debug)]
pub struct OptimizationRecommendation {
    pub priority: RecommendationPriority,
    pub category: String,
    pub description: String,
    pub expected_improvement: String,
    pub implementation_effort: ImplementationEffort,
}

#[derive(Debug)]
pub enum RecommendationPriority {
    Low,
    Medium,
    High,
    Critical,
}

#[derive(Debug)]
pub enum ImplementationEffort {
    Minimal,
    Low,
    Medium,
    High,
    VeryHigh,
}
```

## 最佳实践

### 1. 工具选择指南

1. **开发阶段**: 使用静态分析工具进行早期性能评估
2. **调试阶段**: 结合调试器和跟踪工具进行详细分析
3. **测试阶段**: 使用基准测试和压力测试验证性能
4. **生产阶段**: 部署实时监控系统持续观察

### 2. 性能分析流程

1. **建立基准**: 记录优化前的性能基准
2. **识别瓶颈**: 使用分析工具找出性能瓶颈
3. **制定策略**: 基于分析结果制定优化策略
4. **实施优化**: 逐步实施优化措施
5. **验证效果**: 测量优化后的性能改进
6. **持续监控**: 部署监控系统跟踪长期性能

### 3. 工具配置建议

- **采样频率**: 根据系统负载调整采样频率
- **缓冲区大小**: 平衡内存使用和数据完整性
- **过滤条件**: 设置合适的过滤条件减少噪声
- **报告格式**: 选择适合团队的报告格式

## 结论

性能分析工具是优化STM32F4 ADC/DAC系统的重要手段。通过合理选择和配置这些工具，开发者可以：

1. **深入了解系统行为**: 获得详细的性能数据和分析
2. **快速定位问题**: 准确识别性能瓶颈和问题根源
3. **验证优化效果**: 量化优化带来的性能改进
4. **持续监控系统**: 及时发现和解决性能问题

掌握这些工具的使用方法，将大大提高性能优化工作的效率和效果。