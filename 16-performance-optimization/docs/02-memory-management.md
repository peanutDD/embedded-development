# 内存管理优化

本章详细介绍嵌入式Rust开发中的内存管理优化技术，包括静态内存分配、内存池、缓存优化等。

## 1. 内存布局优化

### 1.1 内存映射配置

```rust
// memory.x - 链接器脚本
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 1024K
  RAM : ORIGIN = 0x20000000, LENGTH = 128K
  CCMRAM : ORIGIN = 0x10000000, LENGTH = 64K  /* 核心耦合内存 */
}

/* 栈大小配置 */
_stack_size = 8K;
_heap_size = 16K;

/* 特殊段配置 */
SECTIONS
{
  .fast_code : {
    *(.fast_code*)
  } > CCMRAM AT > FLASH
  
  .dma_buffer (NOLOAD) : {
    *(.dma_buffer*)
  } > RAM
}
```

### 1.2 数据段优化

```rust
// 将频繁访问的代码放入快速内存
#[link_section = ".fast_code"]
fn critical_function() {
    // 关键性能代码
}

// DMA缓冲区对齐
#[repr(C, align(32))]
struct DmaBuffer {
    data: [u8; 1024],
}

#[link_section = ".dma_buffer"]
static mut DMA_BUFFER: DmaBuffer = DmaBuffer {
    data: [0; 1024],
};

// 常量数据优化
#[link_section = ".rodata"]
static LOOKUP_TABLE: [u16; 256] = {
    let mut table = [0u16; 256];
    let mut i = 0;
    while i < 256 {
        table[i] = (i * i) as u16;
        i += 1;
    }
    table
};
```

### 1.3 栈优化

```rust
use cortex_m_rt::{entry, exception};

// 自定义栈大小
#[no_mangle]
pub static _STACK_SIZE: u32 = 4096; // 4KB栈

// 栈使用监控
struct StackMonitor {
    stack_start: *const u8,
    stack_size: usize,
}

impl StackMonitor {
    fn new() -> Self {
        extern "C" {
            static _stack_start: u8;
            static _stack_size: u8;
        }
        
        Self {
            stack_start: unsafe { &_stack_start as *const u8 },
            stack_size: unsafe { &_stack_size as *const u8 as usize },
        }
    }
    
    fn get_usage(&self) -> usize {
        let current_sp = cortex_m::register::msp::read() as *const u8;
        let stack_end = unsafe { self.stack_start.add(self.stack_size) };
        
        if current_sp < stack_end {
            unsafe { stack_end.offset_from(current_sp) as usize }
        } else {
            0
        }
    }
    
    fn get_free_space(&self) -> usize {
        self.stack_size - self.get_usage()
    }
    
    fn check_overflow(&self) -> bool {
        self.get_free_space() < 256 // 预留256字节
    }
}

// 栈溢出检测
#[exception]
unsafe fn HardFault(_frame: &cortex_m_rt::ExceptionFrame) -> ! {
    // 检查是否为栈溢出
    let monitor = StackMonitor::new();
    if monitor.check_overflow() {
        cortex_m_log::println!("Stack overflow detected!");
    }
    
    loop {}
}
```

## 2. 静态内存分配

### 2.1 编译时内存分配

```rust
use heapless::{Vec, String, FnvIndexMap};

// 静态缓冲区
static mut GLOBAL_BUFFER: [u8; 4096] = [0; 4096];

// 静态向量
type StaticVec<T> = Vec<T, 64>;
type StaticString = String<256>;
type StaticMap<K, V> = FnvIndexMap<K, V, 32>;

struct StaticMemoryManager {
    buffer_pool: [Option<&'static mut [u8]>; 16],
    allocation_map: u32, // 位图标记已分配的块
}

impl StaticMemoryManager {
    const BLOCK_SIZE: usize = 256;
    
    fn new() -> Self {
        Self {
            buffer_pool: [None; 16],
            allocation_map: 0,
        }
    }
    
    fn allocate_block(&mut self) -> Option<&'static mut [u8]> {
        // 查找空闲块
        for i in 0..16 {
            if self.allocation_map & (1 << i) == 0 {
                // 标记为已分配
                self.allocation_map |= 1 << i;
                
                // 返回对应的内存块
                unsafe {
                    let start = GLOBAL_BUFFER.as_mut_ptr().add(i * Self::BLOCK_SIZE);
                    Some(core::slice::from_raw_parts_mut(start, Self::BLOCK_SIZE))
                }
            }
        }
        None
    }
    
    fn deallocate_block(&mut self, block: &[u8]) {
        let block_addr = block.as_ptr() as usize;
        let buffer_addr = unsafe { GLOBAL_BUFFER.as_ptr() as usize };
        
        if block_addr >= buffer_addr && block_addr < buffer_addr + GLOBAL_BUFFER.len() {
            let block_index = (block_addr - buffer_addr) / Self::BLOCK_SIZE;
            if block_index < 16 {
                // 标记为空闲
                self.allocation_map &= !(1 << block_index);
            }
        }
    }
    
    fn get_free_blocks(&self) -> u32 {
        (!self.allocation_map).count_ones()
    }
}
```

### 2.2 内存池实现

```rust
use heapless::pool::{Pool, Node, Box};

// 不同大小的内存池
const SMALL_POOL_SIZE: usize = 32;
const MEDIUM_POOL_SIZE: usize = 16;
const LARGE_POOL_SIZE: usize = 8;

static mut SMALL_MEMORY: [Node<[u8; 64]>; SMALL_POOL_SIZE] = [Node::new(); SMALL_POOL_SIZE];
static mut MEDIUM_MEMORY: [Node<[u8; 256]>; MEDIUM_POOL_SIZE] = [Node::new(); MEDIUM_POOL_SIZE];
static mut LARGE_MEMORY: [Node<[u8; 1024]>; LARGE_POOL_SIZE] = [Node::new(); LARGE_POOL_SIZE];

struct MultiSizeMemoryPool {
    small_pool: Pool<[u8; 64]>,
    medium_pool: Pool<[u8; 256]>,
    large_pool: Pool<[u8; 1024]>,
}

impl MultiSizeMemoryPool {
    fn new() -> Self {
        Self {
            small_pool: Pool::new(unsafe { &mut SMALL_MEMORY }),
            medium_pool: Pool::new(unsafe { &mut MEDIUM_MEMORY }),
            large_pool: Pool::new(unsafe { &mut LARGE_MEMORY }),
        }
    }
    
    fn allocate(&mut self, size: usize) -> Option<PoolBox> {
        if size <= 64 {
            self.small_pool.alloc([0u8; 64]).map(PoolBox::Small)
        } else if size <= 256 {
            self.medium_pool.alloc([0u8; 256]).map(PoolBox::Medium)
        } else if size <= 1024 {
            self.large_pool.alloc([0u8; 1024]).map(PoolBox::Large)
        } else {
            None
        }
    }
    
    fn get_stats(&self) -> PoolStats {
        PoolStats {
            small_free: self.small_pool.capacity() - self.small_pool.len(),
            medium_free: self.medium_pool.capacity() - self.medium_pool.len(),
            large_free: self.large_pool.capacity() - self.large_pool.len(),
        }
    }
}

enum PoolBox {
    Small(Box<[u8; 64]>),
    Medium(Box<[u8; 256]>),
    Large(Box<[u8; 1024]>),
}

impl PoolBox {
    fn as_slice(&self) -> &[u8] {
        match self {
            PoolBox::Small(b) => b.as_slice(),
            PoolBox::Medium(b) => b.as_slice(),
            PoolBox::Large(b) => b.as_slice(),
        }
    }
    
    fn as_mut_slice(&mut self) -> &mut [u8] {
        match self {
            PoolBox::Small(b) => b.as_mut_slice(),
            PoolBox::Medium(b) => b.as_mut_slice(),
            PoolBox::Large(b) => b.as_mut_slice(),
        }
    }
}

struct PoolStats {
    small_free: usize,
    medium_free: usize,
    large_free: usize,
}
```

### 2.3 对象池

```rust
// 对象池实现
struct ObjectPool<T, const N: usize> {
    objects: [Option<T>; N],
    free_list: heapless::Vec<usize, N>,
}

impl<T: Default + Clone, const N: usize> ObjectPool<T, N> {
    fn new() -> Self {
        let mut free_list = heapless::Vec::new();
        for i in 0..N {
            free_list.push(i).ok();
        }
        
        Self {
            objects: [(); N].map(|_| Some(T::default())),
            free_list,
        }
    }
    
    fn acquire(&mut self) -> Option<PooledObject<T, N>> {
        if let Some(index) = self.free_list.pop() {
            if let Some(object) = self.objects[index].take() {
                return Some(PooledObject {
                    object,
                    index,
                    pool: self as *mut Self,
                });
            }
        }
        None
    }
    
    fn release(&mut self, index: usize, object: T) {
        if index < N {
            self.objects[index] = Some(object);
            self.free_list.push(index).ok();
        }
    }
    
    fn available(&self) -> usize {
        self.free_list.len()
    }
}

struct PooledObject<T, const N: usize> {
    object: T,
    index: usize,
    pool: *mut ObjectPool<T, N>,
}

impl<T, const N: usize> core::ops::Deref for PooledObject<T, N> {
    type Target = T;
    
    fn deref(&self) -> &Self::Target {
        &self.object
    }
}

impl<T, const N: usize> core::ops::DerefMut for PooledObject<T, N> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.object
    }
}

impl<T, const N: usize> Drop for PooledObject<T, N> {
    fn drop(&mut self) {
        unsafe {
            let pool = &mut *self.pool;
            let object = core::mem::replace(&mut self.object, unsafe { core::mem::zeroed() });
            pool.release(self.index, object);
        }
    }
}
```

## 3. 缓存优化

### 3.1 数据局部性优化

```rust
// 结构体成员重排序
#[repr(C)]
struct OptimizedStruct {
    // 频繁访问的字段放在前面
    active: bool,
    counter: u32,
    
    // 不常访问的字段放在后面
    config: [u8; 64],
    debug_info: [u8; 128],
}

// 数组访问优化
fn process_matrix_row_major(matrix: &[[u32; 1000]; 1000]) -> u32 {
    let mut sum = 0;
    
    // 按行访问，利用缓存局部性
    for row in matrix.iter() {
        for &value in row.iter() {
            sum += value;
        }
    }
    
    sum
}

fn process_matrix_blocked(matrix: &[[u32; 1000]; 1000], block_size: usize) -> u32 {
    let mut sum = 0;
    
    // 分块访问，提高缓存命中率
    for block_row in (0..1000).step_by(block_size) {
        for block_col in (0..1000).step_by(block_size) {
            for row in block_row..core::cmp::min(block_row + block_size, 1000) {
                for col in block_col..core::cmp::min(block_col + block_size, 1000) {
                    sum += matrix[row][col];
                }
            }
        }
    }
    
    sum
}
```

### 3.2 预取优化

```rust
// 软件预取
#[inline(always)]
fn prefetch_read<T>(ptr: *const T) {
    #[cfg(target_arch = "arm")]
    unsafe {
        core::arch::asm!("pld [{}]", in(reg) ptr);
    }
}

#[inline(always)]
fn prefetch_write<T>(ptr: *mut T) {
    #[cfg(target_arch = "arm")]
    unsafe {
        core::arch::asm!("pldw [{}]", in(reg) ptr);
    }
}

// 预取优化的数组处理
fn process_array_with_prefetch(data: &mut [u32]) {
    const PREFETCH_DISTANCE: usize = 8;
    
    for i in 0..data.len() {
        // 预取未来的数据
        if i + PREFETCH_DISTANCE < data.len() {
            prefetch_read(&data[i + PREFETCH_DISTANCE] as *const u32);
        }
        
        // 处理当前数据
        data[i] = data[i].wrapping_mul(2);
    }
}

// 流式访问优化
struct StreamingBuffer<T, const N: usize> {
    buffer: [T; N],
    read_pos: usize,
    write_pos: usize,
}

impl<T: Copy + Default, const N: usize> StreamingBuffer<T, N> {
    fn new() -> Self {
        Self {
            buffer: [T::default(); N],
            read_pos: 0,
            write_pos: 0,
        }
    }
    
    fn write_streaming(&mut self, data: &[T]) {
        for &item in data {
            self.buffer[self.write_pos] = item;
            self.write_pos = (self.write_pos + 1) % N;
            
            // 预取下一个写入位置
            let next_pos = (self.write_pos + 1) % N;
            prefetch_write(&mut self.buffer[next_pos] as *mut T);
        }
    }
    
    fn read_streaming(&mut self, output: &mut [T]) -> usize {
        let mut count = 0;
        
        for item in output.iter_mut() {
            if self.read_pos != self.write_pos && count < output.len() {
                *item = self.buffer[self.read_pos];
                self.read_pos = (self.read_pos + 1) % N;
                count += 1;
                
                // 预取下一个读取位置
                let next_pos = (self.read_pos + 1) % N;
                prefetch_read(&self.buffer[next_pos] as *const T);
            } else {
                break;
            }
        }
        
        count
    }
}
```

### 3.3 内存对齐优化

```rust
// 缓存行对齐
#[repr(C, align(32))] // ARM Cortex-M4缓存行大小通常为32字节
struct CacheAlignedData {
    data: [u8; 32],
}

// 避免伪共享
#[repr(C)]
struct NoFalseSharing {
    counter1: u32,
    _pad1: [u8; 28], // 填充到缓存行边界
    counter2: u32,
    _pad2: [u8; 28],
}

// SIMD对齐
#[repr(C, align(16))]
struct SimdAlignedData {
    data: [f32; 4], // 128位对齐，适合NEON指令
}

// 内存对齐检查
fn check_alignment<T>(ptr: *const T) -> bool {
    let alignment = core::mem::align_of::<T>();
    (ptr as usize) % alignment == 0
}

// 对齐分配器
struct AlignedAllocator<const ALIGN: usize>;

impl<const ALIGN: usize> AlignedAllocator<ALIGN> {
    fn allocate_aligned<T>(&self, buffer: &mut [u8]) -> Option<&mut T> {
        let size = core::mem::size_of::<T>();
        let align = core::cmp::max(core::mem::align_of::<T>(), ALIGN);
        
        if buffer.len() >= size + align - 1 {
            let ptr = buffer.as_mut_ptr();
            let aligned_ptr = ((ptr as usize + align - 1) & !(align - 1)) as *mut T;
            
            if (aligned_ptr as usize) + size <= (ptr as usize) + buffer.len() {
                unsafe { Some(&mut *aligned_ptr) }
            } else {
                None
            }
        } else {
            None
        }
    }
}
```

## 4. DMA内存管理

### 4.1 DMA缓冲区管理

```rust
use stm32f4xx_hal::dma::{Stream, Transfer, MemoryToPeripheral, PeripheralToMemory};

// DMA安全缓冲区
#[repr(C, align(32))]
struct DmaSafeBuffer<const N: usize> {
    data: [u8; N],
}

impl<const N: usize> DmaSafeBuffer<N> {
    fn new() -> Self {
        Self {
            data: [0; N],
        }
    }
    
    fn as_slice(&self) -> &[u8] {
        &self.data
    }
    
    fn as_mut_slice(&mut self) -> &mut [u8] {
        &mut self.data
    }
    
    // 确保缓存一致性
    fn flush_cache(&self) {
        #[cfg(target_arch = "arm")]
        unsafe {
            let start = self.data.as_ptr() as u32;
            let end = start + N as u32;
            
            // 清理数据缓存
            for addr in (start..end).step_by(32) {
                core::arch::asm!("mcr p15, 0, {}, c7, c10, 1", in(reg) addr);
            }
            
            // 数据同步屏障
            core::arch::asm!("dsb");
        }
    }
    
    fn invalidate_cache(&self) {
        #[cfg(target_arch = "arm")]
        unsafe {
            let start = self.data.as_ptr() as u32;
            let end = start + N as u32;
            
            // 无效化数据缓存
            for addr in (start..end).step_by(32) {
                core::arch::asm!("mcr p15, 0, {}, c7, c6, 1", in(reg) addr);
            }
            
            // 数据同步屏障
            core::arch::asm!("dsb");
        }
    }
}

// DMA缓冲区池
struct DmaBufferPool<const N: usize, const COUNT: usize> {
    buffers: [DmaSafeBuffer<N>; COUNT],
    available: heapless::Vec<usize, COUNT>,
}

impl<const N: usize, const COUNT: usize> DmaBufferPool<N, COUNT> {
    fn new() -> Self {
        let mut available = heapless::Vec::new();
        for i in 0..COUNT {
            available.push(i).ok();
        }
        
        Self {
            buffers: [(); COUNT].map(|_| DmaSafeBuffer::new()),
            available,
        }
    }
    
    fn acquire(&mut self) -> Option<&mut DmaSafeBuffer<N>> {
        if let Some(index) = self.available.pop() {
            Some(&mut self.buffers[index])
        } else {
            None
        }
    }
    
    fn release(&mut self, buffer: &DmaSafeBuffer<N>) {
        let buffer_addr = buffer as *const _ as usize;
        let pool_start = self.buffers.as_ptr() as usize;
        let pool_end = pool_start + COUNT * core::mem::size_of::<DmaSafeBuffer<N>>();
        
        if buffer_addr >= pool_start && buffer_addr < pool_end {
            let index = (buffer_addr - pool_start) / core::mem::size_of::<DmaSafeBuffer<N>>();
            self.available.push(index).ok();
        }
    }
}
```

### 4.2 零拷贝DMA

```rust
// 零拷贝DMA传输
struct ZeroCopyDma<T> {
    buffer: &'static mut [T],
    transfer: Option<Transfer</* DMA参数 */>>,
}

impl<T> ZeroCopyDma<T> {
    fn new(buffer: &'static mut [T]) -> Self {
        Self {
            buffer,
            transfer: None,
        }
    }
    
    fn start_transfer(&mut self) -> Result<(), ()> {
        if self.transfer.is_some() {
            return Err(()); // 传输正在进行
        }
        
        // 启动DMA传输
        // self.transfer = Some(dma_transfer);
        Ok(())
    }
    
    fn is_complete(&self) -> bool {
        self.transfer.is_none()
    }
    
    fn wait_complete(&mut self) -> &[T] {
        while !self.is_complete() {
            // 等待传输完成
        }
        self.buffer
    }
}

// 双缓冲DMA
struct DoubleDma<T, const N: usize> {
    buffer_a: DmaSafeBuffer<N>,
    buffer_b: DmaSafeBuffer<N>,
    active_buffer: bool,
    transfer: Option<Transfer</* DMA参数 */>>,
}

impl<T, const N: usize> DoubleDma<T, N> {
    fn new() -> Self {
        Self {
            buffer_a: DmaSafeBuffer::new(),
            buffer_b: DmaSafeBuffer::new(),
            active_buffer: true,
            transfer: None,
        }
    }
    
    fn get_active_buffer(&mut self) -> &mut DmaSafeBuffer<N> {
        if self.active_buffer {
            &mut self.buffer_a
        } else {
            &mut self.buffer_b
        }
    }
    
    fn get_inactive_buffer(&mut self) -> &mut DmaSafeBuffer<N> {
        if self.active_buffer {
            &mut self.buffer_b
        } else {
            &mut self.buffer_a
        }
    }
    
    fn swap_buffers(&mut self) {
        self.active_buffer = !self.active_buffer;
    }
    
    fn start_continuous_transfer(&mut self) -> Result<(), ()> {
        // 启动连续DMA传输
        Ok(())
    }
}
```

## 5. 内存监控和调试

### 5.1 内存使用统计

```rust
struct MemoryStats {
    total_ram: usize,
    used_ram: usize,
    free_ram: usize,
    stack_usage: usize,
    heap_usage: usize,
    static_usage: usize,
}

impl MemoryStats {
    fn collect() -> Self {
        extern "C" {
            static _sdata: u8;
            static _edata: u8;
            static _sbss: u8;
            static _ebss: u8;
            static _sstack: u8;
            static _estack: u8;
        }
        
        let data_size = unsafe { &_edata as *const u8 as usize - &_sdata as *const u8 as usize };
        let bss_size = unsafe { &_ebss as *const u8 as usize - &_sbss as *const u8 as usize };
        let stack_size = unsafe { &_estack as *const u8 as usize - &_sstack as *const u8 as usize };
        
        let static_usage = data_size + bss_size;
        let current_sp = cortex_m::register::msp::read() as usize;
        let stack_start = unsafe { &_estack as *const u8 as usize };
        let stack_usage = stack_start - current_sp;
        
        Self {
            total_ram: 128 * 1024, // 128KB RAM
            used_ram: static_usage + stack_usage,
            free_ram: 128 * 1024 - static_usage - stack_usage,
            stack_usage,
            heap_usage: 0, // 无堆分配
            static_usage,
        }
    }
    
    fn print_report(&self) {
        cortex_m_log::println!("Memory Usage Report:");
        cortex_m_log::println!("  Total RAM: {} bytes", self.total_ram);
        cortex_m_log::println!("  Used RAM: {} bytes ({}%)", 
            self.used_ram, 
            self.used_ram * 100 / self.total_ram
        );
        cortex_m_log::println!("  Free RAM: {} bytes", self.free_ram);
        cortex_m_log::println!("  Stack Usage: {} bytes", self.stack_usage);
        cortex_m_log::println!("  Static Usage: {} bytes", self.static_usage);
    }
}

// 内存泄漏检测
struct MemoryLeakDetector {
    allocations: heapless::FnvIndexMap<usize, usize, 64>, // 地址 -> 大小
    total_allocated: usize,
    peak_usage: usize,
}

impl MemoryLeakDetector {
    fn new() -> Self {
        Self {
            allocations: heapless::FnvIndexMap::new(),
            total_allocated: 0,
            peak_usage: 0,
        }
    }
    
    fn record_allocation(&mut self, ptr: usize, size: usize) {
        self.allocations.insert(ptr, size).ok();
        self.total_allocated += size;
        
        if self.total_allocated > self.peak_usage {
            self.peak_usage = self.total_allocated;
        }
    }
    
    fn record_deallocation(&mut self, ptr: usize) {
        if let Some(size) = self.allocations.remove(&ptr) {
            self.total_allocated -= size;
        }
    }
    
    fn check_leaks(&self) -> usize {
        self.allocations.len()
    }
    
    fn get_stats(&self) -> (usize, usize, usize) {
        (self.total_allocated, self.peak_usage, self.allocations.len())
    }
}
```

### 5.2 内存保护

```rust
// 内存边界检查
struct BoundedBuffer<const N: usize> {
    data: [u8; N],
    len: usize,
    canary: u32,
}

impl<const N: usize> BoundedBuffer<N> {
    const CANARY_VALUE: u32 = 0xDEADBEEF;
    
    fn new() -> Self {
        Self {
            data: [0; N],
            len: 0,
            canary: Self::CANARY_VALUE,
        }
    }
    
    fn write(&mut self, data: &[u8]) -> Result<(), ()> {
        self.check_integrity()?;
        
        if self.len + data.len() > N {
            return Err(());
        }
        
        self.data[self.len..self.len + data.len()].copy_from_slice(data);
        self.len += data.len();
        
        Ok(())
    }
    
    fn read(&self, offset: usize, len: usize) -> Result<&[u8], ()> {
        self.check_integrity()?;
        
        if offset + len > self.len {
            return Err(());
        }
        
        Ok(&self.data[offset..offset + len])
    }
    
    fn check_integrity(&self) -> Result<(), ()> {
        if self.canary != Self::CANARY_VALUE {
            cortex_m_log::println!("Buffer corruption detected!");
            Err(())
        } else {
            Ok(())
        }
    }
}

// 内存访问保护
struct ProtectedMemory<T> {
    data: T,
    checksum: u32,
}

impl<T> ProtectedMemory<T> 
where
    T: AsRef<[u8]> + AsMut<[u8]>,
{
    fn new(data: T) -> Self {
        let checksum = Self::calculate_checksum(data.as_ref());
        Self { data, checksum }
    }
    
    fn get(&self) -> Result<&T, ()> {
        if self.verify_checksum() {
            Ok(&self.data)
        } else {
            Err(())
        }
    }
    
    fn get_mut(&mut self) -> Result<&mut T, ()> {
        if self.verify_checksum() {
            Ok(&mut self.data)
        } else {
            Err(())
        }
    }
    
    fn update_checksum(&mut self) {
        self.checksum = Self::calculate_checksum(self.data.as_ref());
    }
    
    fn verify_checksum(&self) -> bool {
        self.checksum == Self::calculate_checksum(self.data.as_ref())
    }
    
    fn calculate_checksum(data: &[u8]) -> u32 {
        data.iter().fold(0u32, |acc, &byte| {
            acc.wrapping_mul(31).wrapping_add(byte as u32)
        })
    }
}
```

## 6. 性能测试

### 6.1 内存性能基准测试

```rust
use cortex_m::peripheral::DWT;

struct MemoryBenchmark;

impl MemoryBenchmark {
    fn run_all_tests() {
        Self::test_sequential_access();
        Self::test_random_access();
        Self::test_cache_performance();
        Self::test_dma_performance();
    }
    
    fn test_sequential_access() {
        const SIZE: usize = 1024;
        let mut buffer = [0u8; SIZE];
        
        let start = DWT::cycle_count();
        
        // 顺序写入
        for i in 0..SIZE {
            buffer[i] = (i % 256) as u8;
        }
        
        let write_cycles = DWT::cycle_count() - start;
        
        let start = DWT::cycle_count();
        
        // 顺序读取
        let mut sum = 0u32;
        for &byte in buffer.iter() {
            sum = sum.wrapping_add(byte as u32);
        }
        
        let read_cycles = DWT::cycle_count() - start;
        
        cortex_m_log::println!(
            "Sequential Access - Write: {} cycles, Read: {} cycles, Sum: {}",
            write_cycles, read_cycles, sum
        );
    }
    
    fn test_random_access() {
        const SIZE: usize = 1024;
        let mut buffer = [0u8; SIZE];
        let mut rng = SimpleRng::new(12345);
        
        let start = DWT::cycle_count();
        
        // 随机写入
        for _ in 0..SIZE {
            let index = rng.next() % SIZE;
            buffer[index] = (rng.next() % 256) as u8;
        }
        
        let write_cycles = DWT::cycle_count() - start;
        
        let start = DWT::cycle_count();
        
        // 随机读取
        let mut sum = 0u32;
        for _ in 0..SIZE {
            let index = rng.next() % SIZE;
            sum = sum.wrapping_add(buffer[index] as u32);
        }
        
        let read_cycles = DWT::cycle_count() - start;
        
        cortex_m_log::println!(
            "Random Access - Write: {} cycles, Read: {} cycles, Sum: {}",
            write_cycles, read_cycles, sum
        );
    }
    
    fn test_cache_performance() {
        const SIZE: usize = 2048;
        let buffer = [0u8; SIZE];
        
        // 测试不同步长的访问模式
        for stride in [1, 2, 4, 8, 16, 32, 64].iter() {
            let start = DWT::cycle_count();
            
            let mut sum = 0u32;
            let mut i = 0;
            while i < SIZE {
                sum = sum.wrapping_add(buffer[i] as u32);
                i += stride;
            }
            
            let cycles = DWT::cycle_count() - start;
            
            cortex_m_log::println!(
                "Cache Test - Stride {}: {} cycles, Sum: {}",
                stride, cycles, sum
            );
        }
    }
    
    fn test_dma_performance() {
        // DMA性能测试需要实际的DMA配置
        cortex_m_log::println!("DMA performance test would require actual DMA setup");
    }
}

// 简单随机数生成器
struct SimpleRng {
    state: u32,
}

impl SimpleRng {
    fn new(seed: u32) -> Self {
        Self { state: seed }
    }
    
    fn next(&mut self) -> usize {
        self.state = self.state.wrapping_mul(1103515245).wrapping_add(12345);
        (self.state >> 16) as usize
    }
}
```

## 总结

内存管理优化是嵌入式系统性能优化的核心。通过合理的内存布局、静态分配、缓存优化和DMA管理，可以显著提升系统性能和可靠性。

关键要点：
1. 优化内存布局和数据结构对齐
2. 使用静态内存分配避免碎片化
3. 实现高效的内存池管理
4. 利用缓存局部性优化数据访问
5. 正确管理DMA缓冲区和缓存一致性
6. 实施内存保护和泄漏检测
7. 进行性能基准测试和监控