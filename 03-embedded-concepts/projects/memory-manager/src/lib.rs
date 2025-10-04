#![no_std]
#![deny(unsafe_op_in_unsafe_fn)]
#![warn(missing_docs)]

//! # 嵌入式内存管理器
//! 
//! 本库提供了多种适用于嵌入式系统的内存管理策略和分配器实现。
//! 
//! ## 特性
//! 
//! - 多种内存分配器：Buddy、Slab、Pool、Stack
//! - 内存使用统计和监控
//! - 内存泄漏检测
//! - 零分配数据结构
//! - 实时性保证
//! 
//! ## 使用示例
//! 
//! ```rust,no_run
//! use memory_manager::{BuddyAllocator, MemoryManager};
//! 
//! // 创建内存管理器
//! let mut manager = MemoryManager::new();
//! 
//! // 使用Buddy分配器
//! let mut allocator = BuddyAllocator::new(&mut manager);
//! 
//! // 分配内存
//! let ptr = allocator.allocate(1024).unwrap();
//! 
//! // 释放内存
//! allocator.deallocate(ptr);
//! ```

pub mod allocators;
pub mod memory;
pub mod utils;

// 重新导出主要类型
pub use allocators::{
    buddy::BuddyAllocator,
    slab::SlabAllocator,
    pool::PoolAllocator,
    stack::StackAllocator,
};

pub use memory::{
    manager::MemoryManager,
    tracker::MemoryTracker,
    statistics::MemoryStatistics,
};

pub use utils::{
    alignment::{align_up, align_down, is_aligned},
    bitmap::Bitmap,
};

/// 内存分配错误类型
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MemoryError {
    /// 内存不足
    OutOfMemory,
    /// 无效的大小参数
    InvalidSize,
    /// 无效的对齐参数
    InvalidAlignment,
    /// 空指针
    NullPointer,
    /// 重复释放
    DoubleFree,
    /// 内存损坏
    Corruption,
    /// 分配器未初始化
    NotInitialized,
}

impl core::fmt::Display for MemoryError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            MemoryError::OutOfMemory => write!(f, "Out of memory"),
            MemoryError::InvalidSize => write!(f, "Invalid size"),
            MemoryError::InvalidAlignment => write!(f, "Invalid alignment"),
            MemoryError::NullPointer => write!(f, "Null pointer"),
            MemoryError::DoubleFree => write!(f, "Double free"),
            MemoryError::Corruption => write!(f, "Memory corruption"),
            MemoryError::NotInitialized => write!(f, "Allocator not initialized"),
        }
    }
}

/// 内存分配结果类型
pub type MemoryResult<T> = Result<T, MemoryError>;

/// 内存块描述符
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MemoryBlock {
    /// 内存块起始地址
    pub ptr: *mut u8,
    /// 内存块大小
    pub size: usize,
    /// 内存对齐要求
    pub alignment: usize,
}

impl MemoryBlock {
    /// 创建新的内存块描述符
    pub const fn new(ptr: *mut u8, size: usize, alignment: usize) -> Self {
        Self { ptr, size, alignment }
    }
    
    /// 检查内存块是否有效
    pub fn is_valid(&self) -> bool {
        !self.ptr.is_null() && 
        self.size > 0 && 
        self.alignment > 0 && 
        self.alignment.is_power_of_two() &&
        is_aligned(self.ptr as usize, self.alignment)
    }
    
    /// 获取内存块的结束地址
    pub fn end_ptr(&self) -> *mut u8 {
        unsafe { self.ptr.add(self.size) }
    }
    
    /// 检查是否包含指定地址
    pub fn contains(&self, ptr: *const u8) -> bool {
        ptr >= self.ptr as *const u8 && ptr < self.end_ptr() as *const u8
    }
}

unsafe impl Send for MemoryBlock {}
unsafe impl Sync for MemoryBlock {}

/// 内存分配器特征
pub trait Allocator {
    /// 分配指定大小的内存
    fn allocate(&mut self, size: usize) -> MemoryResult<MemoryBlock>;
    
    /// 分配指定大小和对齐的内存
    fn allocate_aligned(&mut self, size: usize, alignment: usize) -> MemoryResult<MemoryBlock>;
    
    /// 释放内存块
    fn deallocate(&mut self, block: MemoryBlock) -> MemoryResult<()>;
    
    /// 重新分配内存
    fn reallocate(&mut self, block: MemoryBlock, new_size: usize) -> MemoryResult<MemoryBlock> {
        // 默认实现：分配新内存，复制数据，释放旧内存
        let new_block = self.allocate(new_size)?;
        
        if !block.ptr.is_null() && block.size > 0 {
            let copy_size = core::cmp::min(block.size, new_size);
            unsafe {
                core::ptr::copy_nonoverlapping(block.ptr, new_block.ptr, copy_size);
            }
            self.deallocate(block)?;
        }
        
        Ok(new_block)
    }
    
    /// 获取分配器统计信息
    fn statistics(&self) -> MemoryStatistics;
    
    /// 检查分配器状态
    fn check_integrity(&self) -> MemoryResult<()>;
}

/// 零分配分配器特征
/// 
/// 用于需要确定性内存分配的实时系统
pub trait ZeroAllocAllocator {
    /// 预分配内存池
    fn preallocate(&mut self, count: usize, size: usize) -> MemoryResult<()>;
    
    /// 从预分配池中获取内存
    fn get_preallocated(&mut self, size: usize) -> MemoryResult<MemoryBlock>;
    
    /// 返回内存到预分配池
    fn return_preallocated(&mut self, block: MemoryBlock) -> MemoryResult<()>;
    
    /// 获取预分配池状态
    fn pool_status(&self) -> PoolStatus;
}

/// 内存池状态
#[derive(Debug, Clone, Copy)]
pub struct PoolStatus {
    /// 总块数
    pub total_blocks: usize,
    /// 可用块数
    pub available_blocks: usize,
    /// 已使用块数
    pub used_blocks: usize,
    /// 块大小
    pub block_size: usize,
}

impl PoolStatus {
    /// 计算使用率
    pub fn utilization(&self) -> f32 {
        if self.total_blocks == 0 {
            0.0
        } else {
            self.used_blocks as f32 / self.total_blocks as f32
        }
    }
    
    /// 检查是否已满
    pub fn is_full(&self) -> bool {
        self.available_blocks == 0
    }
    
    /// 检查是否为空
    pub fn is_empty(&self) -> bool {
        self.used_blocks == 0
    }
}

/// 内存分配策略
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AllocationStrategy {
    /// 首次适应
    FirstFit,
    /// 最佳适应
    BestFit,
    /// 最差适应
    WorstFit,
    /// 下次适应
    NextFit,
}

/// 内存管理器配置
#[derive(Debug, Clone)]
pub struct MemoryManagerConfig {
    /// 堆大小
    pub heap_size: usize,
    /// 最大分配数量
    pub max_allocations: usize,
    /// 是否启用统计
    pub enable_statistics: bool,
    /// 是否启用内存保护
    pub enable_protection: bool,
    /// 分配策略
    pub allocation_strategy: AllocationStrategy,
    /// 内存对齐要求
    pub default_alignment: usize,
}

impl Default for MemoryManagerConfig {
    fn default() -> Self {
        Self {
            heap_size: 64 * 1024,  // 64KB
            max_allocations: 1000,
            enable_statistics: true,
            enable_protection: false,
            allocation_strategy: AllocationStrategy::FirstFit,
            default_alignment: core::mem::align_of::<usize>(),
        }
    }
}

/// 全局内存管理器实例
/// 
/// 注意：在多线程环境中使用时需要额外的同步机制
pub struct GlobalAllocator {
    inner: Option<MemoryManager>,
}

impl GlobalAllocator {
    /// 创建新的全局分配器
    pub const fn new() -> Self {
        Self { inner: None }
    }
    
    /// 初始化全局分配器
    pub fn init(&mut self, config: MemoryManagerConfig) -> MemoryResult<()> {
        if self.inner.is_some() {
            return Err(MemoryError::NotInitialized);
        }
        
        let manager = MemoryManager::new(config)?;
        self.inner = Some(manager);
        Ok(())
    }
    
    /// 获取全局分配器实例
    pub fn instance(&mut self) -> MemoryResult<&mut MemoryManager> {
        self.inner.as_mut().ok_or(MemoryError::NotInitialized)
    }
}

// 静态全局分配器实例
static mut GLOBAL_ALLOCATOR: GlobalAllocator = GlobalAllocator::new();

/// 初始化全局内存管理器
pub fn init_global_allocator(config: MemoryManagerConfig) -> MemoryResult<()> {
    unsafe {
        GLOBAL_ALLOCATOR.init(config)
    }
}

/// 获取全局内存管理器实例
pub fn global_allocator() -> MemoryResult<&'static mut MemoryManager> {
    unsafe {
        GLOBAL_ALLOCATOR.instance()
    }
}

/// 便利宏：分配内存
#[macro_export]
macro_rules! alloc {
    ($size:expr) => {
        $crate::global_allocator()?.allocate($size)
    };
    ($size:expr, $align:expr) => {
        $crate::global_allocator()?.allocate_aligned($size, $align)
    };
}

/// 便利宏：释放内存
#[macro_export]
macro_rules! dealloc {
    ($block:expr) => {
        $crate::global_allocator()?.deallocate($block)
    };
}

/// 便利宏：重新分配内存
#[macro_export]
macro_rules! realloc {
    ($block:expr, $new_size:expr) => {
        $crate::global_allocator()?.reallocate($block, $new_size)
    };
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_memory_block() {
        let ptr = 0x1000 as *mut u8;
        let block = MemoryBlock::new(ptr, 1024, 8);
        
        assert_eq!(block.ptr, ptr);
        assert_eq!(block.size, 1024);
        assert_eq!(block.alignment, 8);
        assert_eq!(block.end_ptr(), unsafe { ptr.add(1024) });
    }

    #[test]
    fn test_memory_block_contains() {
        let ptr = 0x1000 as *mut u8;
        let block = MemoryBlock::new(ptr, 1024, 8);
        
        assert!(block.contains(0x1000 as *const u8));
        assert!(block.contains(0x1200 as *const u8));
        assert!(!block.contains(0x1400 as *const u8));
        assert!(!block.contains(0x0800 as *const u8));
    }

    #[test]
    fn test_pool_status() {
        let status = PoolStatus {
            total_blocks: 100,
            available_blocks: 30,
            used_blocks: 70,
            block_size: 64,
        };
        
        assert_eq!(status.utilization(), 0.7);
        assert!(!status.is_full());
        assert!(!status.is_empty());
    }

    #[test]
    fn test_memory_manager_config_default() {
        let config = MemoryManagerConfig::default();
        
        assert_eq!(config.heap_size, 64 * 1024);
        assert_eq!(config.max_allocations, 1000);
        assert!(config.enable_statistics);
        assert!(!config.enable_protection);
        assert_eq!(config.allocation_strategy, AllocationStrategy::FirstFit);
    }
}