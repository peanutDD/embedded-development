# RTOS内存管理

## 概述

内存管理是RTOS的核心组件之一，负责系统内存的分配、释放和保护。在嵌入式系统中，内存资源通常有限，因此需要高效、可靠的内存管理机制。

## 内存管理基础

### 内存布局

```rust
/// 内存区域类型
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MemoryRegionType {
    Code,           // 代码段
    Data,           // 数据段
    Bss,            // BSS段
    Stack,          // 栈区
    Heap,           // 堆区
    Device,         // 设备内存
    Reserved,       // 保留区域
}

/// 内存区域描述符
#[derive(Debug, Clone)]
pub struct MemoryRegion {
    pub start_addr: usize,
    pub size: usize,
    pub region_type: MemoryRegionType,
    pub permissions: MemoryPermissions,
    pub name: String<32>,
}

/// 内存权限
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MemoryPermissions {
    pub readable: bool,
    pub writable: bool,
    pub executable: bool,
    pub cacheable: bool,
}

/// 内存布局管理器
pub struct MemoryLayout {
    regions: Vec<MemoryRegion, 16>,
    total_memory: usize,
    used_memory: usize,
}

impl MemoryLayout {
    pub fn new() -> Self {
        Self {
            regions: Vec::new(),
            total_memory: 0,
            used_memory: 0,
        }
    }
    
    /// 添加内存区域
    pub fn add_region(
        &mut self,
        region: MemoryRegion,
    ) -> Result<(), MemoryError> {
        // 检查地址重叠
        for existing in &self.regions {
            if self.regions_overlap(&region, existing) {
                return Err(MemoryError::AddressOverlap);
            }
        }
        
        self.total_memory += region.size;
        self.regions.push(region)
            .map_err(|_| MemoryError::OutOfMemory)?;
        
        Ok(())
    }
    
    /// 检查区域重叠
    fn regions_overlap(&self, region1: &MemoryRegion, region2: &MemoryRegion) -> bool {
        let end1 = region1.start_addr + region1.size;
        let end2 = region2.start_addr + region2.size;
        
        !(end1 <= region2.start_addr || end2 <= region1.start_addr)
    }
    
    /// 查找包含地址的区域
    pub fn find_region(&self, addr: usize) -> Option<&MemoryRegion> {
        self.regions.iter().find(|region| {
            addr >= region.start_addr && addr < region.start_addr + region.size
        })
    }
    
    /// 获取内存使用统计
    pub fn get_memory_stats(&self) -> MemoryStats {
        MemoryStats {
            total_memory: self.total_memory,
            used_memory: self.used_memory,
            free_memory: self.total_memory - self.used_memory,
            fragmentation_ratio: self.calculate_fragmentation(),
        }
    }
    
    /// 计算内存碎片率
    fn calculate_fragmentation(&self) -> f32 {
        // 简化的碎片率计算
        if self.total_memory == 0 {
            0.0
        } else {
            (self.used_memory as f32 / self.total_memory as f32) * 100.0
        }
    }
}

/// 内存统计信息
#[derive(Debug, Clone, Copy)]
pub struct MemoryStats {
    pub total_memory: usize,
    pub used_memory: usize,
    pub free_memory: usize,
    pub fragmentation_ratio: f32,
}
```

### 内存分配器

```rust
/// 内存分配器特征
pub trait MemoryAllocator {
    /// 分配内存
    fn allocate(&mut self, size: usize, alignment: usize) -> Result<*mut u8, MemoryError>;
    
    /// 释放内存
    fn deallocate(&mut self, ptr: *mut u8, size: usize) -> Result<(), MemoryError>;
    
    /// 重新分配内存
    fn reallocate(
        &mut self,
        ptr: *mut u8,
        old_size: usize,
        new_size: usize,
        alignment: usize,
    ) -> Result<*mut u8, MemoryError>;
    
    /// 获取分配器统计信息
    fn get_stats(&self) -> AllocatorStats;
}

/// 分配器统计信息
#[derive(Debug, Clone, Copy, Default)]
pub struct AllocatorStats {
    pub total_allocations: u32,
    pub total_deallocations: u32,
    pub current_allocations: u32,
    pub bytes_allocated: usize,
    pub bytes_deallocated: usize,
    pub peak_memory_usage: usize,
    pub allocation_failures: u32,
}

/// 内存错误类型
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MemoryError {
    OutOfMemory,
    InvalidPointer,
    AddressOverlap,
    AlignmentError,
    AccessViolation,
    DoubleFree,
    CorruptedMetadata,
}
```

## 堆内存管理

### 简单堆分配器

```rust
/// 简单堆分配器
pub struct SimpleHeapAllocator {
    heap_start: *mut u8,
    heap_size: usize,
    next_free: *mut u8,
    stats: AllocatorStats,
}

impl SimpleHeapAllocator {
    /// 创建新的堆分配器
    pub fn new(heap_start: *mut u8, heap_size: usize) -> Self {
        Self {
            heap_start,
            heap_size,
            next_free: heap_start,
            stats: AllocatorStats::default(),
        }
    }
    
    /// 检查指针是否在堆范围内
    fn is_valid_heap_pointer(&self, ptr: *mut u8) -> bool {
        let heap_end = unsafe { self.heap_start.add(self.heap_size) };
        ptr >= self.heap_start && ptr < heap_end
    }
    
    /// 对齐地址
    fn align_up(addr: usize, alignment: usize) -> usize {
        (addr + alignment - 1) & !(alignment - 1)
    }
}

impl MemoryAllocator for SimpleHeapAllocator {
    fn allocate(&mut self, size: usize, alignment: usize) -> Result<*mut u8, MemoryError> {
        if size == 0 {
            return Err(MemoryError::InvalidPointer);
        }
        
        // 对齐当前指针
        let aligned_addr = Self::align_up(self.next_free as usize, alignment);
        let aligned_ptr = aligned_addr as *mut u8;
        
        // 检查是否有足够空间
        let heap_end = unsafe { self.heap_start.add(self.heap_size) };
        let end_ptr = unsafe { aligned_ptr.add(size) };
        
        if end_ptr > heap_end {
            self.stats.allocation_failures += 1;
            return Err(MemoryError::OutOfMemory);
        }
        
        // 更新统计信息
        self.stats.total_allocations += 1;
        self.stats.current_allocations += 1;
        self.stats.bytes_allocated += size;
        
        let current_usage = aligned_ptr as usize - self.heap_start as usize + size;
        if current_usage > self.stats.peak_memory_usage {
            self.stats.peak_memory_usage = current_usage;
        }
        
        // 更新下一个空闲位置
        self.next_free = end_ptr;
        
        Ok(aligned_ptr)
    }
    
    fn deallocate(&mut self, ptr: *mut u8, size: usize) -> Result<(), MemoryError> {
        if !self.is_valid_heap_pointer(ptr) {
            return Err(MemoryError::InvalidPointer);
        }
        
        // 简单堆分配器不支持单独释放，只能重置整个堆
        // 在实际实现中，这里应该有更复杂的逻辑
        
        self.stats.total_deallocations += 1;
        self.stats.current_allocations = self.stats.current_allocations.saturating_sub(1);
        self.stats.bytes_deallocated += size;
        
        Ok(())
    }
    
    fn reallocate(
        &mut self,
        ptr: *mut u8,
        old_size: usize,
        new_size: usize,
        alignment: usize,
    ) -> Result<*mut u8, MemoryError> {
        if new_size == 0 {
            self.deallocate(ptr, old_size)?;
            return Ok(core::ptr::null_mut());
        }
        
        let new_ptr = self.allocate(new_size, alignment)?;
        
        if !ptr.is_null() {
            // 复制旧数据
            let copy_size = old_size.min(new_size);
            unsafe {
                core::ptr::copy_nonoverlapping(ptr, new_ptr, copy_size);
            }
            self.deallocate(ptr, old_size)?;
        }
        
        Ok(new_ptr)
    }
    
    fn get_stats(&self) -> AllocatorStats {
        self.stats
    }
}
```

### 链表堆分配器

```rust
/// 空闲块头部
#[repr(C)]
struct FreeBlockHeader {
    size: usize,
    next: *mut FreeBlockHeader,
}

/// 已分配块头部
#[repr(C)]
struct AllocatedBlockHeader {
    size: usize,
    magic: u32, // 用于检测内存损坏
}

const MAGIC_NUMBER: u32 = 0xDEADBEEF;

/// 链表堆分配器
pub struct LinkedListAllocator {
    free_list: *mut FreeBlockHeader,
    heap_start: *mut u8,
    heap_size: usize,
    stats: AllocatorStats,
}

impl LinkedListAllocator {
    /// 创建新的链表分配器
    pub fn new(heap_start: *mut u8, heap_size: usize) -> Self {
        let mut allocator = Self {
            free_list: heap_start as *mut FreeBlockHeader,
            heap_start,
            heap_size,
            stats: AllocatorStats::default(),
        };
        
        // 初始化空闲列表
        unsafe {
            (*allocator.free_list).size = heap_size;
            (*allocator.free_list).next = core::ptr::null_mut();
        }
        
        allocator
    }
    
    /// 查找合适的空闲块
    fn find_free_block(&mut self, size: usize) -> Option<*mut FreeBlockHeader> {
        let mut current = self.free_list;
        let mut prev: *mut *mut FreeBlockHeader = &mut self.free_list;
        
        while !current.is_null() {
            unsafe {
                if (*current).size >= size {
                    // 从空闲列表中移除
                    *prev = (*current).next;
                    return Some(current);
                }
                prev = &mut (*current).next;
                current = (*current).next;
            }
        }
        
        None
    }
    
    /// 分割块
    fn split_block(&mut self, block: *mut FreeBlockHeader, size: usize) {
        unsafe {
            let block_size = (*block).size;
            let remaining_size = block_size - size - core::mem::size_of::<AllocatedBlockHeader>();
            
            if remaining_size > core::mem::size_of::<FreeBlockHeader>() {
                // 创建新的空闲块
                let new_free_block = (block as *mut u8)
                    .add(size + core::mem::size_of::<AllocatedBlockHeader>())
                    as *mut FreeBlockHeader;
                
                (*new_free_block).size = remaining_size;
                (*new_free_block).next = self.free_list;
                self.free_list = new_free_block;
            }
        }
    }
    
    /// 合并相邻的空闲块
    fn coalesce_free_blocks(&mut self) {
        // 简化实现：在实际系统中需要更复杂的合并逻辑
        // 这里只是示例
    }
}

impl MemoryAllocator for LinkedListAllocator {
    fn allocate(&mut self, size: usize, alignment: usize) -> Result<*mut u8, MemoryError> {
        if size == 0 {
            return Err(MemoryError::InvalidPointer);
        }
        
        let aligned_size = Self::align_up(size, alignment);
        let total_size = aligned_size + core::mem::size_of::<AllocatedBlockHeader>();
        
        if let Some(block) = self.find_free_block(total_size) {
            // 分割块（如果需要）
            self.split_block(block, aligned_size);
            
            // 设置分配块头部
            let header = block as *mut AllocatedBlockHeader;
            unsafe {
                (*header).size = aligned_size;
                (*header).magic = MAGIC_NUMBER;
            }
            
            // 更新统计信息
            self.stats.total_allocations += 1;
            self.stats.current_allocations += 1;
            self.stats.bytes_allocated += aligned_size;
            
            let data_ptr = unsafe {
                (header as *mut u8).add(core::mem::size_of::<AllocatedBlockHeader>())
            };
            
            Ok(data_ptr)
        } else {
            self.stats.allocation_failures += 1;
            Err(MemoryError::OutOfMemory)
        }
    }
    
    fn deallocate(&mut self, ptr: *mut u8, _size: usize) -> Result<(), MemoryError> {
        if ptr.is_null() {
            return Ok(());
        }
        
        // 获取块头部
        let header = unsafe {
            (ptr as *mut AllocatedBlockHeader)
                .sub(1)
        };
        
        // 验证魔数
        unsafe {
            if (*header).magic != MAGIC_NUMBER {
                return Err(MemoryError::CorruptedMetadata);
            }
        }
        
        // 将块添加回空闲列表
        let free_block = header as *mut FreeBlockHeader;
        unsafe {
            (*free_block).size = (*header).size + core::mem::size_of::<AllocatedBlockHeader>();
            (*free_block).next = self.free_list;
        }
        self.free_list = free_block;
        
        // 尝试合并相邻块
        self.coalesce_free_blocks();
        
        // 更新统计信息
        self.stats.total_deallocations += 1;
        self.stats.current_allocations = self.stats.current_allocations.saturating_sub(1);
        
        Ok(())
    }
    
    fn reallocate(
        &mut self,
        ptr: *mut u8,
        old_size: usize,
        new_size: usize,
        alignment: usize,
    ) -> Result<*mut u8, MemoryError> {
        if new_size == 0 {
            self.deallocate(ptr, old_size)?;
            return Ok(core::ptr::null_mut());
        }
        
        if ptr.is_null() {
            return self.allocate(new_size, alignment);
        }
        
        let new_ptr = self.allocate(new_size, alignment)?;
        
        // 复制数据
        let copy_size = old_size.min(new_size);
        unsafe {
            core::ptr::copy_nonoverlapping(ptr, new_ptr, copy_size);
        }
        
        self.deallocate(ptr, old_size)?;
        Ok(new_ptr)
    }
    
    fn get_stats(&self) -> AllocatorStats {
        self.stats
    }
}

impl LinkedListAllocator {
    fn align_up(addr: usize, alignment: usize) -> usize {
        (addr + alignment - 1) & !(alignment - 1)
    }
}
```

## 内存池管理

### 固定大小内存池

```rust
/// 固定大小内存池
pub struct FixedSizePool {
    block_size: usize,
    block_count: usize,
    free_blocks: Vec<*mut u8, 64>,
    pool_start: *mut u8,
    pool_size: usize,
    stats: PoolStats,
}

/// 内存池统计信息
#[derive(Debug, Clone, Copy, Default)]
pub struct PoolStats {
    pub total_blocks: usize,
    pub free_blocks: usize,
    pub allocated_blocks: usize,
    pub peak_usage: usize,
    pub allocation_count: u32,
    pub deallocation_count: u32,
}

impl FixedSizePool {
    /// 创建新的固定大小内存池
    pub fn new(
        pool_start: *mut u8,
        pool_size: usize,
        block_size: usize,
    ) -> Result<Self, MemoryError> {
        if block_size == 0 || pool_size < block_size {
            return Err(MemoryError::InvalidPointer);
        }
        
        let block_count = pool_size / block_size;
        let mut free_blocks = Vec::new();
        
        // 初始化空闲块列表
        for i in 0..block_count {
            let block_ptr = unsafe { pool_start.add(i * block_size) };
            free_blocks.push(block_ptr).map_err(|_| MemoryError::OutOfMemory)?;
        }
        
        Ok(Self {
            block_size,
            block_count,
            free_blocks,
            pool_start,
            pool_size,
            stats: PoolStats {
                total_blocks: block_count,
                free_blocks: block_count,
                allocated_blocks: 0,
                peak_usage: 0,
                allocation_count: 0,
                deallocation_count: 0,
            },
        })
    }
    
    /// 分配一个块
    pub fn allocate(&mut self) -> Result<*mut u8, MemoryError> {
        if let Some(block) = self.free_blocks.pop() {
            self.stats.free_blocks -= 1;
            self.stats.allocated_blocks += 1;
            self.stats.allocation_count += 1;
            
            if self.stats.allocated_blocks > self.stats.peak_usage {
                self.stats.peak_usage = self.stats.allocated_blocks;
            }
            
            Ok(block)
        } else {
            Err(MemoryError::OutOfMemory)
        }
    }
    
    /// 释放一个块
    pub fn deallocate(&mut self, ptr: *mut u8) -> Result<(), MemoryError> {
        // 验证指针是否属于这个池
        if !self.is_valid_block(ptr) {
            return Err(MemoryError::InvalidPointer);
        }
        
        // 检查是否已经在空闲列表中
        if self.free_blocks.iter().any(|&block| block == ptr) {
            return Err(MemoryError::DoubleFree);
        }
        
        self.free_blocks.push(ptr).map_err(|_| MemoryError::OutOfMemory)?;
        self.stats.free_blocks += 1;
        self.stats.allocated_blocks -= 1;
        self.stats.deallocation_count += 1;
        
        Ok(())
    }
    
    /// 验证块指针是否有效
    fn is_valid_block(&self, ptr: *mut u8) -> bool {
        let pool_end = unsafe { self.pool_start.add(self.pool_size) };
        
        if ptr < self.pool_start || ptr >= pool_end {
            return false;
        }
        
        // 检查对齐
        let offset = ptr as usize - self.pool_start as usize;
        offset % self.block_size == 0
    }
    
    /// 获取池统计信息
    pub fn get_stats(&self) -> PoolStats {
        self.stats
    }
    
    /// 获取使用率
    pub fn get_utilization(&self) -> f32 {
        if self.stats.total_blocks == 0 {
            0.0
        } else {
            (self.stats.allocated_blocks as f32 / self.stats.total_blocks as f32) * 100.0
        }
    }
}
```

### 多大小内存池管理器

```rust
/// 内存池配置
#[derive(Debug, Clone, Copy)]
pub struct PoolConfig {
    pub block_size: usize,
    pub block_count: usize,
}

/// 多大小内存池管理器
pub struct MultiSizePoolManager {
    pools: Vec<(usize, FixedSizePool), 8>, // (block_size, pool)
    fallback_allocator: Option<Box<dyn MemoryAllocator>>,
    stats: MultiPoolStats,
}

/// 多池统计信息
#[derive(Debug, Clone, Default)]
pub struct MultiPoolStats {
    pub total_pools: usize,
    pub total_allocations: u32,
    pub total_deallocations: u32,
    pub fallback_allocations: u32,
    pub pool_hits: u32,
    pub pool_misses: u32,
}

impl MultiSizePoolManager {
    /// 创建新的多大小池管理器
    pub fn new() -> Self {
        Self {
            pools: Vec::new(),
            fallback_allocator: None,
            stats: MultiPoolStats::default(),
        }
    }
    
    /// 添加内存池
    pub fn add_pool(
        &mut self,
        block_size: usize,
        pool: FixedSizePool,
    ) -> Result<(), MemoryError> {
        // 按块大小排序插入
        let insert_pos = self.pools.iter().position(|(size, _)| *size > block_size)
            .unwrap_or(self.pools.len());
        
        self.pools.insert(insert_pos, (block_size, pool))
            .map_err(|_| MemoryError::OutOfMemory)?;
        
        self.stats.total_pools += 1;
        Ok(())
    }
    
    /// 设置后备分配器
    pub fn set_fallback_allocator(&mut self, allocator: Box<dyn MemoryAllocator>) {
        self.fallback_allocator = Some(allocator);
    }
    
    /// 分配内存
    pub fn allocate(&mut self, size: usize) -> Result<*mut u8, MemoryError> {
        self.stats.total_allocations += 1;
        
        // 查找合适的池
        for (block_size, pool) in &mut self.pools {
            if size <= *block_size {
                match pool.allocate() {
                    Ok(ptr) => {
                        self.stats.pool_hits += 1;
                        return Ok(ptr);
                    }
                    Err(MemoryError::OutOfMemory) => {
                        // 继续尝试下一个池
                        continue;
                    }
                    Err(e) => return Err(e),
                }
            }
        }
        
        // 使用后备分配器
        if let Some(ref mut fallback) = self.fallback_allocator {
            self.stats.fallback_allocations += 1;
            self.stats.pool_misses += 1;
            fallback.allocate(size, core::mem::align_of::<usize>())
        } else {
            self.stats.pool_misses += 1;
            Err(MemoryError::OutOfMemory)
        }
    }
    
    /// 释放内存
    pub fn deallocate(&mut self, ptr: *mut u8, size: usize) -> Result<(), MemoryError> {
        self.stats.total_deallocations += 1;
        
        // 尝试在池中释放
        for (block_size, pool) in &mut self.pools {
            if size <= *block_size && pool.is_valid_block(ptr) {
                return pool.deallocate(ptr);
            }
        }
        
        // 使用后备分配器释放
        if let Some(ref mut fallback) = self.fallback_allocator {
            fallback.deallocate(ptr, size)
        } else {
            Err(MemoryError::InvalidPointer)
        }
    }
    
    /// 获取统计信息
    pub fn get_stats(&self) -> MultiPoolStats {
        self.stats.clone()
    }
    
    /// 获取池使用情况
    pub fn get_pool_utilization(&self) -> Vec<(usize, f32), 8> {
        let mut utilization = Vec::new();
        
        for (block_size, pool) in &self.pools {
            let util = pool.get_utilization();
            let _ = utilization.push((*block_size, util));
        }
        
        utilization
    }
}
```

## 栈内存管理

### 任务栈管理

```rust
/// 栈信息
#[derive(Debug, Clone)]
pub struct StackInfo {
    pub stack_base: *mut u8,
    pub stack_size: usize,
    pub stack_pointer: *mut u8,
    pub guard_size: usize,
    pub task_id: u32,
}

/// 栈管理器
pub struct StackManager {
    stacks: Vec<StackInfo, 32>,
    stack_pool: Vec<(*mut u8, usize), 32>, // (base, size)
    default_stack_size: usize,
    guard_enabled: bool,
    stats: StackStats,
}

/// 栈统计信息
#[derive(Debug, Clone, Copy, Default)]
pub struct StackStats {
    pub total_stacks: u32,
    pub active_stacks: u32,
    pub stack_overflows: u32,
    pub max_stack_usage: usize,
    pub total_stack_memory: usize,
}

impl StackManager {
    /// 创建栈管理器
    pub fn new(default_stack_size: usize, guard_enabled: bool) -> Self {
        Self {
            stacks: Vec::new(),
            stack_pool: Vec::new(),
            default_stack_size,
            guard_enabled,
            stats: StackStats::default(),
        }
    }
    
    /// 分配栈
    pub fn allocate_stack(
        &mut self,
        task_id: u32,
        stack_size: Option<usize>,
    ) -> Result<StackInfo, MemoryError> {
        let size = stack_size.unwrap_or(self.default_stack_size);
        let guard_size = if self.guard_enabled { 4096 } else { 0 };
        let total_size = size + guard_size;
        
        // 尝试从池中获取
        let stack_base = if let Some(index) = self.stack_pool.iter()
            .position(|(_, pool_size)| *pool_size >= total_size) {
            let (base, _) = self.stack_pool.swap_remove(index);
            base
        } else {
            // 分配新栈
            self.allocate_new_stack(total_size)?
        };
        
        // 设置栈保护（如果启用）
        if self.guard_enabled {
            self.setup_stack_guard(stack_base, guard_size)?;
        }
        
        let stack_info = StackInfo {
            stack_base,
            stack_size: size,
            stack_pointer: unsafe { stack_base.add(total_size) },
            guard_size,
            task_id,
        };
        
        self.stacks.push(stack_info.clone())
            .map_err(|_| MemoryError::OutOfMemory)?;
        
        self.stats.total_stacks += 1;
        self.stats.active_stacks += 1;
        self.stats.total_stack_memory += total_size;
        
        Ok(stack_info)
    }
    
    /// 释放栈
    pub fn deallocate_stack(&mut self, task_id: u32) -> Result<(), MemoryError> {
        if let Some(index) = self.stacks.iter().position(|s| s.task_id == task_id) {
            let stack_info = self.stacks.swap_remove(index);
            
            // 清理栈保护
            if self.guard_enabled {
                self.cleanup_stack_guard(stack_info.stack_base, stack_info.guard_size)?;
            }
            
            // 返回到池中
            let total_size = stack_info.stack_size + stack_info.guard_size;
            self.stack_pool.push((stack_info.stack_base, total_size))
                .map_err(|_| MemoryError::OutOfMemory)?;
            
            self.stats.active_stacks -= 1;
            Ok(())
        } else {
            Err(MemoryError::InvalidPointer)
        }
    }
    
    /// 检查栈溢出
    pub fn check_stack_overflow(&mut self, task_id: u32, current_sp: *mut u8) -> bool {
        if let Some(stack_info) = self.stacks.iter().find(|s| s.task_id == task_id) {
            let stack_limit = unsafe { stack_info.stack_base.add(stack_info.guard_size) };
            
            if current_sp <= stack_limit {
                self.stats.stack_overflows += 1;
                return true;
            }
            
            // 更新最大栈使用量
            let usage = stack_info.stack_pointer as usize - current_sp as usize;
            if usage > self.stats.max_stack_usage {
                self.stats.max_stack_usage = usage;
            }
        }
        
        false
    }
    
    /// 分配新栈
    fn allocate_new_stack(&self, size: usize) -> Result<*mut u8, MemoryError> {
        // 在实际实现中，这里应该调用系统的内存分配函数
        // 这里只是示例
        let layout = core::alloc::Layout::from_size_align(size, 4096)
            .map_err(|_| MemoryError::AlignmentError)?;
        
        unsafe {
            let ptr = core::alloc::alloc(layout);
            if ptr.is_null() {
                Err(MemoryError::OutOfMemory)
            } else {
                Ok(ptr)
            }
        }
    }
    
    /// 设置栈保护
    fn setup_stack_guard(&self, base: *mut u8, guard_size: usize) -> Result<(), MemoryError> {
        // 在实际实现中，这里应该设置内存保护属性
        // 使栈保护区域不可访问
        Ok(())
    }
    
    /// 清理栈保护
    fn cleanup_stack_guard(&self, base: *mut u8, guard_size: usize) -> Result<(), MemoryError> {
        // 清理栈保护设置
        Ok(())
    }
    
    /// 获取栈统计信息
    pub fn get_stats(&self) -> StackStats {
        self.stats
    }
    
    /// 获取任务栈信息
    pub fn get_stack_info(&self, task_id: u32) -> Option<&StackInfo> {
        self.stacks.iter().find(|s| s.task_id == task_id)
    }
}
```

## 内存保护

### 内存保护单元(MPU)

```rust
/// 内存保护区域
#[derive(Debug, Clone)]
pub struct ProtectionRegion {
    pub region_id: u8,
    pub base_address: usize,
    pub size: usize,
    pub permissions: MemoryPermissions,
    pub enabled: bool,
}

/// 内存保护单元管理器
pub struct MPUManager {
    regions: Vec<ProtectionRegion, 16>,
    max_regions: u8,
    enabled: bool,
    violation_handler: Option<fn(usize, MemoryPermissions)>,
}

impl MPUManager {
    /// 创建MPU管理器
    pub fn new(max_regions: u8) -> Self {
        Self {
            regions: Vec::new(),
            max_regions,
            enabled: false,
            violation_handler: None,
        }
    }
    
    /// 配置保护区域
    pub fn configure_region(
        &mut self,
        region_id: u8,
        base_address: usize,
        size: usize,
        permissions: MemoryPermissions,
    ) -> Result<(), MemoryError> {
        if region_id >= self.max_regions {
            return Err(MemoryError::InvalidPointer);
        }
        
        // 检查地址对齐
        if !self.is_address_aligned(base_address, size) {
            return Err(MemoryError::AlignmentError);
        }
        
        let region = ProtectionRegion {
            region_id,
            base_address,
            size,
            permissions,
            enabled: true,
        };
        
        // 更新或添加区域
        if let Some(existing) = self.regions.iter_mut().find(|r| r.region_id == region_id) {
            *existing = region;
        } else {
            self.regions.push(region).map_err(|_| MemoryError::OutOfMemory)?;
        }
        
        // 配置硬件MPU
        self.configure_hardware_mpu(region_id, base_address, size, permissions)?;
        
        Ok(())
    }
    
    /// 启用MPU
    pub fn enable(&mut self) -> Result<(), MemoryError> {
        self.enabled = true;
        self.enable_hardware_mpu()
    }
    
    /// 禁用MPU
    pub fn disable(&mut self) -> Result<(), MemoryError> {
        self.enabled = false;
        self.disable_hardware_mpu()
    }
    
    /// 检查内存访问权限
    pub fn check_access(
        &self,
        address: usize,
        access_type: MemoryAccessType,
    ) -> Result<(), MemoryError> {
        if !self.enabled {
            return Ok(());
        }
        
        for region in &self.regions {
            if region.enabled && 
               address >= region.base_address && 
               address < region.base_address + region.size {
                
                let allowed = match access_type {
                    MemoryAccessType::Read => region.permissions.readable,
                    MemoryAccessType::Write => region.permissions.writable,
                    MemoryAccessType::Execute => region.permissions.executable,
                };
                
                if allowed {
                    return Ok(());
                } else {
                    return Err(MemoryError::AccessViolation);
                }
            }
        }
        
        // 默认拒绝访问
        Err(MemoryError::AccessViolation)
    }
    
    /// 设置违规处理器
    pub fn set_violation_handler(&mut self, handler: fn(usize, MemoryPermissions)) {
        self.violation_handler = Some(handler);
    }
    
    /// 处理内存访问违规
    pub fn handle_violation(&self, address: usize, attempted_access: MemoryPermissions) {
        if let Some(handler) = self.violation_handler {
            handler(address, attempted_access);
        }
    }
    
    /// 检查地址对齐
    fn is_address_aligned(&self, address: usize, size: usize) -> bool {
        // MPU通常要求地址和大小按特定边界对齐
        let alignment = size.next_power_of_two();
        address % alignment == 0 && size.is_power_of_two()
    }
    
    /// 配置硬件MPU
    fn configure_hardware_mpu(
        &self,
        region_id: u8,
        base_address: usize,
        size: usize,
        permissions: MemoryPermissions,
    ) -> Result<(), MemoryError> {
        // 在实际实现中，这里应该配置硬件MPU寄存器
        // 这里只是示例
        Ok(())
    }
    
    /// 启用硬件MPU
    fn enable_hardware_mpu(&self) -> Result<(), MemoryError> {
        // 启用硬件MPU
        Ok(())
    }
    
    /// 禁用硬件MPU
    fn disable_hardware_mpu(&self) -> Result<(), MemoryError> {
        // 禁用硬件MPU
        Ok(())
    }
}

/// 内存访问类型
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MemoryAccessType {
    Read,
    Write,
    Execute,
}
```

## 内存调试和诊断

### 内存泄漏检测

```rust
/// 内存分配记录
#[derive(Debug, Clone)]
struct AllocationRecord {
    ptr: *mut u8,
    size: usize,
    timestamp: u64,
    caller: usize, // 调用者地址
}

/// 内存泄漏检测器
pub struct MemoryLeakDetector {
    allocations: Vec<AllocationRecord, 256>,
    enabled: bool,
    leak_threshold_ms: u64,
    stats: LeakDetectorStats,
}

/// 泄漏检测统计
#[derive(Debug, Clone, Copy, Default)]
pub struct LeakDetectorStats {
    pub tracked_allocations: u32,
    pub detected_leaks: u32,
    pub false_positives: u32,
    pub total_leaked_bytes: usize,
}

impl MemoryLeakDetector {
    pub fn new(leak_threshold_ms: u64) -> Self {
        Self {
            allocations: Vec::new(),
            enabled: false,
            leak_threshold_ms,
            stats: LeakDetectorStats::default(),
        }
    }
    
    /// 启用泄漏检测
    pub fn enable(&mut self) {
        self.enabled = true;
    }
    
    /// 禁用泄漏检测
    pub fn disable(&mut self) {
        self.enabled = false;
    }
    
    /// 记录内存分配
    pub fn record_allocation(
        &mut self,
        ptr: *mut u8,
        size: usize,
        caller: usize,
    ) -> Result<(), MemoryError> {
        if !self.enabled {
            return Ok(());
        }
        
        let record = AllocationRecord {
            ptr,
            size,
            timestamp: get_current_time() as u64,
            caller,
        };
        
        self.allocations.push(record)
            .map_err(|_| MemoryError::OutOfMemory)?;
        
        self.stats.tracked_allocations += 1;
        Ok(())
    }
    
    /// 记录内存释放
    pub fn record_deallocation(&mut self, ptr: *mut u8) -> Result<(), MemoryError> {
        if !self.enabled {
            return Ok(());
        }
        
        if let Some(index) = self.allocations.iter().position(|r| r.ptr == ptr) {
            self.allocations.swap_remove(index);
            Ok(())
        } else {
            Err(MemoryError::InvalidPointer)
        }
    }
    
    /// 检查内存泄漏
    pub fn check_leaks(&mut self) -> Vec<AllocationRecord, 32> {
        let mut leaks = Vec::new();
        let current_time = get_current_time() as u64;
        
        for record in &self.allocations {
            if current_time - record.timestamp > self.leak_threshold_ms {
                self.stats.detected_leaks += 1;
                self.stats.total_leaked_bytes += record.size;
                let _ = leaks.push(record.clone());
            }
        }
        
        leaks
    }
    
    /// 获取统计信息
    pub fn get_stats(&self) -> LeakDetectorStats {
        self.stats
    }
    
    /// 清理检测器
    pub fn clear(&mut self) {
        self.allocations.clear();
        self.stats = LeakDetectorStats::default();
    }
}
```

### 内存损坏检测

```rust
/// 内存损坏检测器
pub struct MemoryCorruptionDetector {
    canary_value: u32,
    check_frequency: u32,
    check_counter: u32,
    protected_regions: Vec<ProtectedRegion, 16>,
    stats: CorruptionStats,
}

/// 受保护的内存区域
#[derive(Debug, Clone)]
struct ProtectedRegion {
    start: *mut u8,
    size: usize,
    checksum: u32,
    last_check: u64,
}

/// 损坏检测统计
#[derive(Debug, Clone, Copy, Default)]
pub struct CorruptionStats {
    pub total_checks: u32,
    pub corruptions_detected: u32,
    pub false_alarms: u32,
    pub regions_monitored: usize,
}

impl MemoryCorruptionDetector {
    pub fn new(canary_value: u32, check_frequency: u32) -> Self {
        Self {
            canary_value,
            check_frequency,
            check_counter: 0,
            protected_regions: Vec::new(),
            stats: CorruptionStats::default(),
        }
    }
    
    /// 添加受保护区域
    pub fn add_protected_region(
        &mut self,
        start: *mut u8,
        size: usize,
    ) -> Result<(), MemoryError> {
        let checksum = self.calculate_checksum(start, size);
        
        let region = ProtectedRegion {
            start,
            size,
            checksum,
            last_check: get_current_time() as u64,
        };
        
        self.protected_regions.push(region)
            .map_err(|_| MemoryError::OutOfMemory)?;
        
        self.stats.regions_monitored += 1;
        Ok(())
    }
    
    /// 检查内存完整性
    pub fn check_integrity(&mut self) -> Vec<usize, 16> {
        self.check_counter += 1;
        
        if self.check_counter % self.check_frequency != 0 {
            return Vec::new();
        }
        
        let mut corrupted_regions = Vec::new();
        let current_time = get_current_time() as u64;
        
        for (index, region) in self.protected_regions.iter_mut().enumerate() {
            self.stats.total_checks += 1;
            
            let current_checksum = self.calculate_checksum(region.start, region.size);
            
            if current_checksum != region.checksum {
                self.stats.corruptions_detected += 1;
                let _ = corrupted_regions.push(index);
            }
            
            region.last_check = current_time;
        }
        
        corrupted_regions
    }
    
    /// 计算校验和
    fn calculate_checksum(&self, ptr: *mut u8, size: usize) -> u32 {
        let mut checksum = 0u32;
        
        unsafe {
            for i in 0..size {
                let byte = *ptr.add(i);
                checksum = checksum.wrapping_add(byte as u32);
                checksum = checksum.rotate_left(1);
            }
        }
        
        checksum ^ self.canary_value
    }
    
    /// 获取统计信息
    pub fn get_stats(&self) -> CorruptionStats {
        self.stats
    }
}
```

## 总结

内存管理是RTOS的关键组件，本文档涵盖了：

1. **内存布局管理** - 系统内存区域的组织和管理
2. **堆内存分配** - 动态内存分配算法和实现
3. **内存池管理** - 固定大小和多大小内存池
4. **栈内存管理** - 任务栈的分配和保护
5. **内存保护** - MPU和访问控制机制
6. **内存调试** - 泄漏检测和损坏检测

选择合适的内存管理策略需要考虑系统的实时性要求、内存大小限制和安全性需求。