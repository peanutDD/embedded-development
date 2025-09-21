# 代码优化

本章介绍嵌入式Rust开发中的代码优化技术，包括编译器优化、算法优化、内存优化等方面。

## 1. 编译器优化

### 1.1 优化级别配置

```toml
# Cargo.toml
[profile.release]
opt-level = "s"        # 优化代码大小
lto = true            # 链接时优化
codegen-units = 1     # 单个代码生成单元
panic = "abort"       # 减少panic处理代码
overflow-checks = false # 禁用溢出检查
```

### 1.2 目标特定优化

```toml
# .cargo/config.toml
[target.thumbv7em-none-eabihf]
rustflags = [
  "-C", "target-cpu=cortex-m4",
  "-C", "target-feature=+fp-armv8d16",
  "-C", "link-arg=-Tlink.x",
]
```

### 1.3 编译器提示

```rust
// 内联函数
#[inline(always)]
fn fast_multiply(a: u32, b: u32) -> u32 {
    a.wrapping_mul(b)
}

// 分支预测
#[cold]
fn error_handler() {
    // 错误处理代码
}

#[inline]
fn likely_path(condition: bool) -> bool {
    if likely(condition) {
        true
    } else {
        false
    }
}

// 使用likely/unlikely宏
macro_rules! likely {
    ($e:expr) => {
        #[allow(unused_unsafe)]
        unsafe {
            core::intrinsics::likely($e)
        }
    };
}

macro_rules! unlikely {
    ($e:expr) => {
        #[allow(unused_unsafe)]
        unsafe {
            core::intrinsics::unlikely($e)
        }
    };
}
```

## 2. 算法优化

### 2.1 快速数学运算

```rust
use micromath::F32Ext;

// 快速平方根
fn fast_sqrt(x: f32) -> f32 {
    x.sqrt()
}

// 快速三角函数
fn fast_sin_cos(angle: f32) -> (f32, f32) {
    (angle.sin(), angle.cos())
}

// 位操作优化
fn is_power_of_two(n: u32) -> bool {
    n != 0 && (n & (n - 1)) == 0
}

fn next_power_of_two(n: u32) -> u32 {
    if n == 0 {
        1
    } else {
        1 << (32 - n.leading_zeros())
    }
}

// 快速除法（除以2的幂）
fn divide_by_power_of_two(value: u32, power: u8) -> u32 {
    value >> power
}

// 快速乘法（乘以2的幂）
fn multiply_by_power_of_two(value: u32, power: u8) -> u32 {
    value << power
}
```

### 2.2 查找表优化

```rust
// 正弦查找表
const SIN_TABLE: [f32; 256] = {
    let mut table = [0.0; 256];
    let mut i = 0;
    while i < 256 {
        table[i] = libm::sinf(2.0 * core::f32::consts::PI * i as f32 / 256.0);
        i += 1;
    }
    table
};

fn fast_sin(angle: f32) -> f32 {
    let normalized = (angle / (2.0 * core::f32::consts::PI)) % 1.0;
    let index = (normalized * 256.0) as usize % 256;
    SIN_TABLE[index]
}

// CRC查找表
const CRC_TABLE: [u32; 256] = {
    let mut table = [0u32; 256];
    let mut i = 0;
    while i < 256 {
        let mut crc = i as u32;
        let mut j = 0;
        while j < 8 {
            if crc & 1 != 0 {
                crc = (crc >> 1) ^ 0xEDB88320;
            } else {
                crc >>= 1;
            }
            j += 1;
        }
        table[i] = crc;
        i += 1;
    }
    table
};

fn fast_crc32(data: &[u8]) -> u32 {
    let mut crc = 0xFFFFFFFF;
    for &byte in data {
        let index = ((crc ^ byte as u32) & 0xFF) as usize;
        crc = (crc >> 8) ^ CRC_TABLE[index];
    }
    !crc
}
```

### 2.3 循环优化

```rust
// 循环展开
fn sum_array_unrolled(arr: &[u32]) -> u32 {
    let mut sum = 0;
    let mut i = 0;
    
    // 4路展开
    while i + 3 < arr.len() {
        sum += arr[i] + arr[i + 1] + arr[i + 2] + arr[i + 3];
        i += 4;
    }
    
    // 处理剩余元素
    while i < arr.len() {
        sum += arr[i];
        i += 1;
    }
    
    sum
}

// SIMD优化（如果支持）
#[cfg(target_feature = "neon")]
fn simd_add(a: &[f32], b: &[f32], result: &mut [f32]) {
    use core::arch::arm::*;
    
    assert_eq!(a.len(), b.len());
    assert_eq!(a.len(), result.len());
    assert_eq!(a.len() % 4, 0);
    
    for i in (0..a.len()).step_by(4) {
        unsafe {
            let va = vld1q_f32(a.as_ptr().add(i));
            let vb = vld1q_f32(b.as_ptr().add(i));
            let vr = vaddq_f32(va, vb);
            vst1q_f32(result.as_mut_ptr().add(i), vr);
        }
    }
}
```

## 3. 内存优化

### 3.1 数据结构优化

```rust
// 紧凑的数据结构
#[repr(C, packed)]
struct CompactSensor {
    id: u8,
    value: u16,
    status: u8,
}

// 对齐优化
#[repr(C)]
struct AlignedData {
    flag: bool,      // 1 byte
    _pad1: [u8; 3],  // 3 bytes padding
    value: u32,      // 4 bytes, aligned
    timestamp: u64,  // 8 bytes, aligned
}

// 使用位字段
struct StatusFlags {
    data: u8,
}

impl StatusFlags {
    fn new() -> Self {
        Self { data: 0 }
    }
    
    fn set_ready(&mut self, ready: bool) {
        if ready {
            self.data |= 0x01;
        } else {
            self.data &= !0x01;
        }
    }
    
    fn is_ready(&self) -> bool {
        self.data & 0x01 != 0
    }
    
    fn set_error(&mut self, error: bool) {
        if error {
            self.data |= 0x02;
        } else {
            self.data &= !0x02;
        }
    }
    
    fn has_error(&self) -> bool {
        self.data & 0x02 != 0
    }
}
```

### 3.2 内存池管理

```rust
use heapless::pool::{Pool, Node};

// 内存池配置
const POOL_SIZE: usize = 32;
static mut MEMORY: [Node<[u8; 64]>; POOL_SIZE] = [Node::new(); POOL_SIZE];

struct MemoryManager {
    pool: Pool<[u8; 64]>,
}

impl MemoryManager {
    fn new() -> Self {
        Self {
            pool: Pool::new(unsafe { &mut MEMORY }),
        }
    }
    
    fn allocate(&mut self) -> Option<heapless::pool::Box<[u8; 64]>> {
        self.pool.alloc([0u8; 64])
    }
    
    fn get_available(&self) -> usize {
        self.pool.capacity() - self.pool.len()
    }
}

// 零拷贝缓冲区
struct ZeroCopyBuffer<'a> {
    data: &'a mut [u8],
    len: usize,
}

impl<'a> ZeroCopyBuffer<'a> {
    fn new(buffer: &'a mut [u8]) -> Self {
        Self {
            data: buffer,
            len: 0,
        }
    }
    
    fn write(&mut self, data: &[u8]) -> Result<(), ()> {
        if self.len + data.len() <= self.data.len() {
            self.data[self.len..self.len + data.len()].copy_from_slice(data);
            self.len += data.len();
            Ok(())
        } else {
            Err(())
        }
    }
    
    fn as_slice(&self) -> &[u8] {
        &self.data[..self.len]
    }
}
```

### 3.3 栈优化

```rust
// 避免大型栈分配
fn process_large_data() -> Result<(), ()> {
    // 不好的做法：大型栈数组
    // let large_array = [0u8; 4096];
    
    // 好的做法：使用静态缓冲区
    static mut BUFFER: [u8; 4096] = [0; 4096];
    let buffer = unsafe { &mut BUFFER };
    
    // 处理数据
    for i in 0..buffer.len() {
        buffer[i] = (i % 256) as u8;
    }
    
    Ok(())
}

// 尾递归优化
fn factorial_iterative(n: u32) -> u32 {
    let mut result = 1;
    for i in 1..=n {
        result *= i;
    }
    result
}

// 避免递归，使用迭代
fn fibonacci_iterative(n: u32) -> u32 {
    if n <= 1 {
        return n;
    }
    
    let mut a = 0;
    let mut b = 1;
    
    for _ in 2..=n {
        let temp = a + b;
        a = b;
        b = temp;
    }
    
    b
}
```

## 4. I/O优化

### 4.1 DMA优化

```rust
use stm32f4xx_hal::{
    dma::{Stream0, StreamsTuple, Transfer, MemoryToPeripheral, PeripheralToMemory},
    pac::DMA2,
    serial::{Tx, Rx},
};

struct OptimizedUart {
    tx_transfer: Option<Transfer<Stream0<DMA2>, Tx<stm32f4xx_hal::pac::USART1>, MemoryToPeripheral, &'static [u8]>>,
    rx_buffer: &'static mut [u8; 256],
}

impl OptimizedUart {
    fn send_dma(&mut self, data: &'static [u8]) -> Result<(), ()> {
        if self.tx_transfer.is_none() {
            // 启动DMA传输
            // self.tx_transfer = Some(transfer);
            Ok(())
        } else {
            Err(()) // DMA忙碌
        }
    }
    
    fn is_tx_complete(&mut self) -> bool {
        if let Some(transfer) = &self.tx_transfer {
            // 检查传输是否完成
            true // 简化实现
        } else {
            true
        }
    }
}

// 批量I/O操作
struct BatchedIO {
    write_buffer: heapless::Vec<u8, 256>,
    read_buffer: heapless::Vec<u8, 256>,
}

impl BatchedIO {
    fn new() -> Self {
        Self {
            write_buffer: heapless::Vec::new(),
            read_buffer: heapless::Vec::new(),
        }
    }
    
    fn queue_write(&mut self, data: &[u8]) -> Result<(), ()> {
        if self.write_buffer.len() + data.len() <= self.write_buffer.capacity() {
            self.write_buffer.extend_from_slice(data).map_err(|_| ())?;
            Ok(())
        } else {
            Err(())
        }
    }
    
    fn flush_writes(&mut self) -> Result<(), ()> {
        if !self.write_buffer.is_empty() {
            // 执行批量写入
            // hardware_write(&self.write_buffer);
            self.write_buffer.clear();
        }
        Ok(())
    }
}
```

### 4.2 缓冲区优化

```rust
use heapless::spsc::{Queue, Producer, Consumer};

// 环形缓冲区
struct RingBuffer<T, const N: usize> {
    buffer: [T; N],
    head: usize,
    tail: usize,
    full: bool,
}

impl<T: Copy + Default, const N: usize> RingBuffer<T, N> {
    fn new() -> Self {
        Self {
            buffer: [T::default(); N],
            head: 0,
            tail: 0,
            full: false,
        }
    }
    
    fn push(&mut self, item: T) -> Result<(), T> {
        if self.full {
            return Err(item);
        }
        
        self.buffer[self.head] = item;
        self.head = (self.head + 1) % N;
        
        if self.head == self.tail {
            self.full = true;
        }
        
        Ok(())
    }
    
    fn pop(&mut self) -> Option<T> {
        if self.is_empty() {
            return None;
        }
        
        let item = self.buffer[self.tail];
        self.tail = (self.tail + 1) % N;
        self.full = false;
        
        Some(item)
    }
    
    fn is_empty(&self) -> bool {
        !self.full && self.head == self.tail
    }
    
    fn is_full(&self) -> bool {
        self.full
    }
    
    fn len(&self) -> usize {
        if self.full {
            N
        } else if self.head >= self.tail {
            self.head - self.tail
        } else {
            N - self.tail + self.head
        }
    }
}

// 双缓冲区
struct DoubleBuffer<T, const N: usize> {
    buffer_a: [T; N],
    buffer_b: [T; N],
    active_buffer: bool, // true = A, false = B
}

impl<T: Copy + Default, const N: usize> DoubleBuffer<T, N> {
    fn new() -> Self {
        Self {
            buffer_a: [T::default(); N],
            buffer_b: [T::default(); N],
            active_buffer: true,
        }
    }
    
    fn get_active_buffer(&mut self) -> &mut [T; N] {
        if self.active_buffer {
            &mut self.buffer_a
        } else {
            &mut self.buffer_b
        }
    }
    
    fn get_inactive_buffer(&mut self) -> &mut [T; N] {
        if self.active_buffer {
            &mut self.buffer_b
        } else {
            &mut self.buffer_a
        }
    }
    
    fn swap_buffers(&mut self) {
        self.active_buffer = !self.active_buffer;
    }
}
```

## 5. 实时优化

### 5.1 中断优化

```rust
use cortex_m::interrupt;

// 快速中断处理
#[interrupt]
fn TIM2() {
    // 最小化中断处理时间
    static mut COUNTER: u32 = 0;
    
    unsafe {
        *COUNTER = COUNTER.wrapping_add(1);
        
        // 清除中断标志
        let tim2 = &*stm32f4xx_hal::pac::TIM2::ptr();
        tim2.sr.modify(|_, w| w.uif().clear_bit());
    }
    
    // 延迟处理到主循环
    // set_pending_flag();
}

// 中断优先级管理
fn configure_interrupt_priorities() {
    use cortex_m::peripheral::NVIC;
    use stm32f4xx_hal::pac::Interrupt;
    
    unsafe {
        // 高优先级：关键实时任务
        NVIC::set_priority(Interrupt::TIM2, 0);
        
        // 中优先级：重要但非关键任务
        NVIC::set_priority(Interrupt::USART1, 1);
        
        // 低优先级：后台任务
        NVIC::set_priority(Interrupt::EXTI0, 2);
    }
}

// 关键段优化
fn critical_section_optimized<F, R>(f: F) -> R
where
    F: FnOnce() -> R,
{
    interrupt::free(|_| {
        // 最小化关键段代码
        f()
    })
}
```

### 5.2 任务调度优化

```rust
// 优先级队列
use heapless::binary_heap::{BinaryHeap, Max};

#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
struct Task {
    priority: u8,
    id: u8,
}

struct TaskScheduler {
    ready_queue: BinaryHeap<Task, Max, 16>,
    current_task: Option<Task>,
}

impl TaskScheduler {
    fn new() -> Self {
        Self {
            ready_queue: BinaryHeap::new(),
            current_task: None,
        }
    }
    
    fn add_task(&mut self, task: Task) -> Result<(), Task> {
        self.ready_queue.push(task)
    }
    
    fn schedule(&mut self) -> Option<Task> {
        if let Some(next_task) = self.ready_queue.pop() {
            let prev_task = self.current_task.replace(next_task);
            
            // 如果有被抢占的任务，重新加入队列
            if let Some(prev) = prev_task {
                if prev.priority < next_task.priority {
                    self.ready_queue.push(prev).ok();
                }
            }
            
            Some(next_task)
        } else {
            None
        }
    }
}

// 时间片管理
struct TimeSliceManager {
    quantum: u32,
    remaining: u32,
}

impl TimeSliceManager {
    fn new(quantum: u32) -> Self {
        Self {
            quantum,
            remaining: quantum,
        }
    }
    
    fn tick(&mut self) -> bool {
        if self.remaining > 0 {
            self.remaining -= 1;
            self.remaining == 0
        } else {
            false
        }
    }
    
    fn reset(&mut self) {
        self.remaining = self.quantum;
    }
}
```

## 6. 性能测试

### 6.1 基准测试

```rust
use cortex_m_rt::entry;
use cortex_m::asm;

#[entry]
fn main() -> ! {
    // 性能测试框架
    run_benchmarks();
    
    loop {
        asm::wfi();
    }
}

fn run_benchmarks() {
    benchmark_math_operations();
    benchmark_memory_operations();
    benchmark_io_operations();
}

fn benchmark_math_operations() {
    let start = get_cycle_count();
    
    // 测试数学运算
    let mut result = 0u32;
    for i in 0..1000 {
        result = result.wrapping_add(fast_multiply(i, i + 1));
    }
    
    let end = get_cycle_count();
    let cycles = end - start;
    
    cortex_m_log::println!("Math operations: {} cycles", cycles);
}

fn benchmark_memory_operations() {
    let mut buffer = [0u8; 1024];
    
    let start = get_cycle_count();
    
    // 测试内存操作
    for i in 0..buffer.len() {
        buffer[i] = (i % 256) as u8;
    }
    
    let end = get_cycle_count();
    let cycles = end - start;
    
    cortex_m_log::println!("Memory operations: {} cycles", cycles);
}

fn get_cycle_count() -> u32 {
    cortex_m::peripheral::DWT::cycle_count()
}

// 性能分析器
struct Profiler {
    start_time: u32,
    samples: heapless::Vec<u32, 100>,
}

impl Profiler {
    fn new() -> Self {
        Self {
            start_time: 0,
            samples: heapless::Vec::new(),
        }
    }
    
    fn start(&mut self) {
        self.start_time = get_cycle_count();
    }
    
    fn end(&mut self) {
        let duration = get_cycle_count() - self.start_time;
        self.samples.push(duration).ok();
    }
    
    fn get_average(&self) -> u32 {
        if self.samples.is_empty() {
            0
        } else {
            self.samples.iter().sum::<u32>() / self.samples.len() as u32
        }
    }
    
    fn get_max(&self) -> u32 {
        self.samples.iter().copied().max().unwrap_or(0)
    }
    
    fn get_min(&self) -> u32 {
        self.samples.iter().copied().min().unwrap_or(0)
    }
}
```

### 6.2 实时性能监控

```rust
struct PerformanceMonitor {
    cpu_usage: u8,
    max_interrupt_latency: u32,
    context_switch_count: u32,
    memory_usage: usize,
}

impl PerformanceMonitor {
    fn new() -> Self {
        Self {
            cpu_usage: 0,
            max_interrupt_latency: 0,
            context_switch_count: 0,
            memory_usage: 0,
        }
    }
    
    fn update_cpu_usage(&mut self, idle_time: u32, total_time: u32) {
        if total_time > 0 {
            self.cpu_usage = ((total_time - idle_time) * 100 / total_time) as u8;
        }
    }
    
    fn record_interrupt_latency(&mut self, latency: u32) {
        if latency > self.max_interrupt_latency {
            self.max_interrupt_latency = latency;
        }
    }
    
    fn increment_context_switches(&mut self) {
        self.context_switch_count = self.context_switch_count.wrapping_add(1);
    }
    
    fn report(&self) {
        cortex_m_log::println!(
            "Performance: CPU={}%, Max Latency={}us, Switches={}",
            self.cpu_usage,
            self.max_interrupt_latency,
            self.context_switch_count
        );
    }
}
```

## 总结

代码优化是嵌入式开发中的关键技能，需要在性能、功耗和代码可维护性之间找到平衡。通过合理使用编译器优化、算法优化、内存管理和实时优化技术，可以显著提升系统性能。

关键要点：
1. 使用适当的编译器优化选项
2. 选择高效的算法和数据结构
3. 优化内存使用和访问模式
4. 最小化中断延迟和关键段
5. 进行性能测试和监控
6. 平衡优化与代码可读性