//! 内存管理器实现

use super::{statistics::MemoryStatistics, tracker::MemoryTracker};
use crate::{Allocator, AllocationStrategy, MemoryBlock, MemoryError, MemoryResult, MemoryManagerConfig};
use core::alloc::Layout;
use core::mem::size_of;
use core::ptr::NonNull;

/// 内存块头部
#[repr(C)]
struct BlockHeader {
    /// 块大小（不包括头部）
    size: usize,
    /// 下一个块的指针
    next: Option<NonNull<BlockHeader>>,
    /// 前一个块的指针
    prev: Option<NonNull<BlockHeader>>,
    /// 是否已分配
    allocated: bool,
    /// 分配标记（用于检测内存损坏）
    canary: u32,
}

const CANARY_VALUE: u32 = 0xDEADBEEF;

impl BlockHeader {
    /// 获取块的总大小（包括头部）
    fn total_size(&self) -> usize {
        self.size + size_of::<BlockHeader>()
    }

    /// 获取块的数据指针
    fn data_ptr(&self) -> *mut u8 {
        (self as *const BlockHeader as *mut u8).add(size_of::<BlockHeader>())
    }

    /// 从数据指针获取块头部
    unsafe fn from_data_ptr(data_ptr: *mut u8) -> *mut BlockHeader {
        data_ptr.sub(size_of::<BlockHeader>()) as *mut BlockHeader
    }

    /// 检查块是否有效（防止内存损坏）
    fn is_valid(&self) -> bool {
        self.canary == CANARY_VALUE
    }

    /// 创建新的块头部
    fn new(size: usize, allocated: bool) -> Self {
        Self {
            size,
            next: None,
            prev: None,
            allocated,
            canary: CANARY_VALUE,
        }
    }
}

/// 内存管理器实现
pub struct MemoryManager {
    /// 堆起始地址
    heap_start: *mut u8,
    /// 堆结束地址
    heap_end: *mut u8,
    /// 堆大小
    heap_size: usize,
    /// 空闲块链表头
    free_list: Option<NonNull<BlockHeader>>,
    /// 分配块数量
    allocation_count: usize,
    /// 最大分配数量
    max_allocations: usize,
    /// 内存跟踪器
    tracker: Option<MemoryTracker>,
    /// 内存统计信息
    statistics: MemoryStatistics,
    /// 分配策略
    strategy: AllocationStrategy,
    /// 默认对齐要求
    default_alignment: usize,
    /// 是否启用统计
    enable_statistics: bool,
    /// 是否启用内存保护
    enable_protection: bool,
}

impl MemoryManager {
    /// 创建新的内存管理器
    pub fn new(config: MemoryManagerConfig) -> MemoryResult<Self> {
        // 验证配置
        if config.heap_size == 0 {
            return Err(MemoryError::InvalidSize);
        }

        if config.default_alignment == 0 || !config.default_alignment.is_power_of_two() {
            return Err(MemoryError::InvalidAlignment);
        }

        // 分配堆内存
        let heap_size = config.heap_size;
        let heap_memory = unsafe {
            core::alloc::alloc(Layout::from_size_align(heap_size, config.default_alignment).unwrap())
        };

        if heap_memory.is_null() {
            return Err(MemoryError::OutOfMemory);
        }

        // 创建初始空闲块
        let heap_start = heap_memory;
        let heap_end = unsafe { heap_start.add(heap_size) };
        let initial_block_ptr = heap_start as *mut BlockHeader;

        unsafe {
            core::ptr::write(
                initial_block_ptr,
                BlockHeader::new(heap_size - size_of::<BlockHeader>(), false),
            );
        }

        let free_list = NonNull::new(initial_block_ptr);

        // 创建内存跟踪器
        let tracker = if config.enable_protection {
            Some(MemoryTracker::new(config.max_allocations))
        } else {
            None
        };

        Ok(Self {
            heap_start,
            heap_end,
            heap_size,
            free_list,
            allocation_count: 0,
            max_allocations: config.max_allocations,
            tracker,
            statistics: MemoryStatistics::new(),
            strategy: config.allocation_strategy,
            default_alignment: config.default_alignment,
            enable_statistics: config.enable_statistics,
            enable_protection: config.enable_protection,
        })
    }

    /// 查找合适的空闲块
    fn find_free_block(&mut self, size: usize, alignment: usize) -> Option<NonNull<BlockHeader>> {
        let mut current = self.free_list;
        let mut best_fit_block: Option<NonNull<BlockHeader>> = None;
        let mut best_fit_size = core::usize::MAX;

        while let Some(mut block) = current {
            let block_ref = unsafe { block.as_mut() };
            
            // 检查块是否有效
            if !block_ref.is_valid() {
                return None; // 内存损坏
            }

            // 计算需要的对齐调整
            let data_ptr = block_ref.data_ptr();
            let align_adjust = if alignment > 1 {
                let misalignment = (data_ptr as usize) % alignment;
                if misalignment == 0 {
                    0
                } else {
                    alignment - misalignment
                }
            } else {
                0
            };

            // 检查块是否足够大（考虑对齐调整）
            let required_size = size + align_adjust + size_of::<BlockHeader>();
            if block_ref.size >= required_size {
                match self.strategy {
                    AllocationStrategy::FirstFit => {
                        // 首次适应策略：返回找到的第一个足够大的块
                        return Some(block);
                    },
                    AllocationStrategy::BestFit => {
                        // 最佳适应策略：记录最小的足够大的块
                        if block_ref.size < best_fit_size {
                            best_fit_size = block_ref.size;
                            best_fit_block = Some(block);
                        }
                    },
                    AllocationStrategy::WorstFit => {
                        // 最差适应策略：记录最大的足够大的块
                        if block_ref.size > best_fit_size {
                            best_fit_size = block_ref.size;
                            best_fit_block = Some(block);
                        }
                    },
                    AllocationStrategy::NextFit => {
                        // TODO: 实现下次适应策略
                        return Some(block);
                    },
                }
            }

            current = block_ref.next;
        }

        // 对于最佳适应和最差适应，返回找到的最佳块
        best_fit_block
    }

    /// 分割块
    fn split_block(&mut self, block: NonNull<BlockHeader>, size: usize) {
        let block_ref = unsafe { block.as_mut() };
        let remaining_size = block_ref.size - size - size_of::<BlockHeader>();

        // 只有当剩余空间足够大时才分割块
        if remaining_size > size_of::<BlockHeader>() + 16 { // 16 字节是最小块大小
            // 创建新的空闲块
            let new_block_ptr = unsafe {
                block_ref.data_ptr().add(size) as *mut BlockHeader
            };

            unsafe {
                core::ptr::write(
                    new_block_ptr,
                    BlockHeader::new(remaining_size, false),
                );
            }

            let new_block = NonNull::new(new_block_ptr).unwrap();
            let new_block_ref = unsafe { new_block.as_mut() };

            // 更新链表
            new_block_ref.next = block_ref.next;
            new_block_ref.prev = Some(block);
            if let Some(next) = block_ref.next {
                unsafe { next.as_mut() }.prev = Some(new_block);
            }
            block_ref.next = Some(new_block);
            block_ref.size = size;
        }
    }

    /// 合并相邻的空闲块
    fn merge_adjacent_blocks(&mut self) {
        let mut current = self.free_list;

        while let Some(mut block) = current {
            let block_ref = unsafe { block.as_mut() };
            
            if let Some(mut next_block) = block_ref.next {
                let next_block_ref = unsafe { next_block.as_mut() };
                
                // 检查两个块是否相邻
                let block_end = unsafe { block_ref.data_ptr().add(block_ref.size) };
                let next_block_start = next_block.as_ptr() as *mut u8;
                
                if block_end == next_block_start {
                    // 合并块
                    block_ref.size += next_block_ref.total_size();
                    block_ref.next = next_block_ref.next;
                    
                    if let Some(next_next_block) = next_block_ref.next {
                        unsafe { next_next_block.as_mut() }.prev = Some(block);
                    }
                    
                    // 不移动到下一个块，因为当前块可能还可以与新的下一个块合并
                    continue;
                }
            }
            
            current = block_ref.next;
        }
    }

    /// 计算可用内存
    pub fn available_memory(&self) -> usize {
        let mut available = 0;
        let mut current = self.free_list;
        
        while let Some(block) = current {
            available += unsafe { block.as_ref() }.size;
            current = unsafe { block.as_ref() }.next;
        }
        
        available
    }

    /// 获取分配块数量
    pub fn allocation_count(&self) -> usize {
        self.allocation_count
    }
}

impl Allocator for MemoryManager {
    fn allocate(&mut self, size: usize) -> MemoryResult<MemoryBlock> {
        self.allocate_aligned(size, self.default_alignment)
    }

    fn allocate_aligned(&mut self, size: usize, alignment: usize) -> MemoryResult<MemoryBlock> {
        // 验证参数
        if size == 0 {
            return Err(MemoryError::InvalidSize);
        }
        
        if alignment == 0 || !alignment.is_power_of_two() {
            return Err(MemoryError::InvalidAlignment);
        }
        
        if self.allocation_count >= self.max_allocations {
            return Err(MemoryError::OutOfMemory);
        }

        // 找到合适的空闲块
        let block = self.find_free_block(size, alignment)
            .ok_or(MemoryError::OutOfMemory)?;

        let block_ref = unsafe { block.as_mut() };
        let data_ptr = block_ref.data_ptr();
        
        // 计算对齐调整
        let align_adjust = if alignment > 1 {
            let misalignment = (data_ptr as usize) % alignment;
            if misalignment == 0 {
                0
            } else {
                alignment - misalignment
            }
        } else {
            0
        };

        // 分割块
        self.split_block(block, size + align_adjust);

        // 标记块为已分配
        block_ref.allocated = true;
        self.allocation_count += 1;

        // 更新统计信息
        if self.enable_statistics {
            self.statistics.total_allocations += 1;
            self.statistics.current_allocated += size;
            if self.statistics.max_allocated < self.statistics.current_allocated {
                self.statistics.max_allocated = self.statistics.current_allocated;
            }
        }

        // 如果启用了内存保护，跟踪分配
        if self.enable_protection {
            if let Some(ref mut tracker) = self.tracker {
                tracker.track_allocation(
                    unsafe { data_ptr.add(align_adjust) },
                    size,
                    alignment
                )?;
            }
        }

        // 创建内存块描述符
        let result_ptr = unsafe { data_ptr.add(align_adjust) };
        Ok(MemoryBlock::new(result_ptr, size, alignment))
    }

    fn deallocate(&mut self, block: MemoryBlock) -> MemoryResult<()> {
        // 验证块
        if !block.is_valid() {
            return Err(MemoryError::NullPointer);
        }

        // 检查指针是否在堆范围内
        if block.ptr < self.heap_start || block.ptr.add(block.size) > self.heap_end {
            return Err(MemoryError::NullPointer);
        }

        // 获取块头部
        let header_ptr = unsafe { BlockHeader::from_data_ptr(block.ptr) };
        let header = NonNull::new(header_ptr)
            .ok_or(MemoryError::NullPointer)?;

        let header_ref = unsafe { header.as_mut() };

        // 检查块是否有效
        if !header_ref.is_valid() {
            return Err(MemoryError::Corruption);
        }

        // 检查块是否已分配
        if !header_ref.allocated {
            return Err(MemoryError::DoubleFree);
        }

        // 如果启用了内存保护，检查是否是有效的分配
        if self.enable_protection {
            if let Some(ref mut tracker) = self.tracker {
                tracker.check_deallocation(block.ptr, block.size)?;
            }
        }

        // 标记块为空闲
        header_ref.allocated = false;
        self.allocation_count -= 1;

        // 更新统计信息
        if self.enable_statistics {
            self.statistics.total_deallocations += 1;
            self.statistics.current_allocated -= block.size;
        }

        // 合并相邻的空闲块
        self.merge_adjacent_blocks();

        Ok(())
    }

    fn reallocate(&mut self, block: MemoryBlock, new_size: usize) -> MemoryResult<MemoryBlock> {
        // 如果新大小为0，释放内存
        if new_size == 0 {
            self.deallocate(block)?;
            return Err(MemoryError::InvalidSize);
        }

        // 如果块为空，分配新内存
        if !block.is_valid() {
            return self.allocate_aligned(new_size, block.alignment);
        }

        // 尝试在当前位置扩展/缩小块
        let header_ptr = unsafe { BlockHeader::from_data_ptr(block.ptr) };
        let header = NonNull::new(header_ptr)
            .ok_or(MemoryError::NullPointer)?;

        let header_ref = unsafe { header.as_mut() };

        // 检查块是否有效
        if !header_ref.is_valid() {
            return Err(MemoryError::Corruption);
        }

        // 检查块是否已分配
        if !header_ref.allocated {
            return Err(MemoryError::NullPointer);
        }

        // 如果新大小小于等于当前大小，直接调整块大小
        if new_size <= block.size {
            return Ok(block);
        }

        // 否则，分配新内存，复制数据，释放旧内存
        let new_block = self.allocate_aligned(new_size, block.alignment)?;
        
        // 复制数据
        unsafe {
            core::ptr::copy_nonoverlapping(
                block.ptr,
                new_block.ptr,
                block.size
            );
        }
        
        // 释放旧内存
        self.deallocate(block)?;
        
        Ok(new_block)
    }

    fn statistics(&self) -> MemoryStatistics {
        self.statistics
    }

    fn check_integrity(&self) -> MemoryResult<()> {
        let mut current = self.free_list;
        let mut last_end = self.heap_start;
        let mut block_count = 0;

        while let Some(block) = current {
            let block_ref = unsafe { block.as_ref() };
            
            // 检查块是否有效
            if !block_ref.is_valid() {
                return Err(MemoryError::Corruption);
            }

            // 检查块是否重叠或有间隙
            let block_start = block.as_ptr() as *mut u8;
            if block_start < last_end {
                return Err(MemoryError::Corruption); // 块重叠
            }
            
            last_end = unsafe { block_ref.data_ptr().add(block_ref.size) };
            block_count += 1;
            
            current = block_ref.next;
        }

        // 检查最后一个块是否到达堆末尾
        if last_end > self.heap_end {
            return Err(MemoryError::Corruption);
        }

        // 检查分配计数
        if self.allocation_count > self.max_allocations {
            return Err(MemoryError::Corruption);
        }

        // 如果启用了内存保护，检查跟踪器
        if self.enable_protection {
            if let Some(ref tracker) = self.tracker {
                tracker.check_integrity()?;
            }
        }

        Ok(())
    }
}

impl Drop for MemoryManager {
    fn drop(&mut self) {
        // 释放堆内存
        let layout = Layout::from_size_align(self.heap_size, self.default_alignment).unwrap();
        unsafe {
            core::alloc::dealloc(self.heap_start, layout);
        }
    }
}