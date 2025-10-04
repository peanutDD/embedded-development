use crate::{Allocator, MemoryBlock, MemoryError, MemoryResult, MemoryStatistics};
use crate::utils::{align_up, is_aligned, Bitmap};
use heapless::Vec;

/// Buddy分配器
/// 
/// 使用伙伴系统算法管理内存，将内存分割成2的幂次大小的块
pub struct BuddyAllocator {
    memory_start: *mut u8,
    memory_size: usize,
    min_block_size: usize,
    max_order: usize,
    free_lists: Vec<Vec<usize, 64>, 16>, // 每个order的空闲块列表
    allocated_blocks: Bitmap,
    statistics: BuddyStatistics,
}

#[derive(Debug, Clone)]
struct BuddyStatistics {
    total_allocations: u32,
    total_deallocations: u32,
    current_allocated_bytes: usize,
    peak_allocated_bytes: usize,
    fragmentation_count: u32,
    merge_count: u32,
}

impl BuddyAllocator {
    /// 创建新的Buddy分配器
    /// 
    /// # 参数
    /// - `memory_start`: 内存起始地址
    /// - `memory_size`: 内存总大小（必须是2的幂次）
    /// - `min_block_size`: 最小块大小（必须是2的幂次）
    pub fn new(memory_start: *mut u8, memory_size: usize, min_block_size: usize) -> MemoryResult<Self> {
        if !memory_size.is_power_of_two() || !min_block_size.is_power_of_two() {
            return Err(MemoryError::InvalidSize);
        }
        
        if memory_size < min_block_size {
            return Err(MemoryError::InvalidSize);
        }
        
        let max_order = (memory_size / min_block_size).trailing_zeros() as usize;
        let total_blocks = memory_size / min_block_size;
        
        let mut allocator = Self {
            memory_start,
            memory_size,
            min_block_size,
            max_order,
            free_lists: Vec::new(),
            allocated_blocks: Bitmap::new(total_blocks),
            statistics: BuddyStatistics {
                total_allocations: 0,
                total_deallocations: 0,
                current_allocated_bytes: 0,
                peak_allocated_bytes: 0,
                fragmentation_count: 0,
                merge_count: 0,
            },
        };
        
        // 初始化空闲列表
        for _ in 0..=max_order {
            allocator.free_lists.push(Vec::new()).map_err(|_| MemoryError::OutOfMemory)?;
        }
        
        // 将整个内存作为一个大块加入最高order的空闲列表
        allocator.free_lists[max_order].push(0).map_err(|_| MemoryError::OutOfMemory)?;
        
        Ok(allocator)
    }
    
    /// 计算所需的order
    fn calculate_order(&self, size: usize) -> usize {
        let aligned_size = align_up(size, self.min_block_size);
        let blocks_needed = aligned_size / self.min_block_size;
        
        if blocks_needed == 0 {
            0
        } else {
            (blocks_needed - 1).leading_zeros() as usize
        }
    }
    
    /// 分割块
    fn split_block(&mut self, order: usize, block_index: usize) -> MemoryResult<()> {
        if order == 0 {
            return Err(MemoryError::InvalidSize);
        }
        
        let block_size = self.min_block_size << order;
        let half_size = block_size >> 1;
        
        // 计算伙伴块的索引
        let buddy_index = block_index + (half_size / self.min_block_size);
        
        // 将两个半块加入下一级空闲列表
        self.free_lists[order - 1].push(block_index).map_err(|_| MemoryError::OutOfMemory)?;
        self.free_lists[order - 1].push(buddy_index).map_err(|_| MemoryError::OutOfMemory)?;
        
        Ok(())
    }
    
    /// 合并块
    fn merge_blocks(&mut self, order: usize, block_index: usize) -> MemoryResult<usize> {
        if order >= self.max_order {
            return Ok(block_index);
        }
        
        let block_size = self.min_block_size << order;
        let buddy_index = block_index ^ (block_size / self.min_block_size);
        
        // 检查伙伴块是否空闲
        if let Some(pos) = self.free_lists[order].iter().position(|&x| x == buddy_index) {
            // 移除伙伴块
            self.free_lists[order].remove(pos);
            
            // 递归合并到更高order
            let merged_index = core::cmp::min(block_index, buddy_index);
            self.statistics.merge_count += 1;
            self.merge_blocks(order + 1, merged_index)
        } else {
            // 伙伴块不空闲，无法合并
            self.free_lists[order].push(block_index).map_err(|_| MemoryError::OutOfMemory)?;
            Ok(block_index)
        }
    }
    
    /// 获取块的物理地址
    fn get_block_address(&self, block_index: usize) -> *mut u8 {
        unsafe {
            self.memory_start.add(block_index * self.min_block_size)
        }
    }
    
    /// 从地址计算块索引
    fn get_block_index(&self, ptr: *mut u8) -> usize {
        let offset = ptr as usize - self.memory_start as usize;
        offset / self.min_block_size
    }
}

impl Allocator for BuddyAllocator {
    fn allocate(&mut self, size: usize) -> MemoryResult<MemoryBlock> {
        self.allocate_aligned(size, self.min_block_size)
    }
    
    fn allocate_aligned(&mut self, size: usize, alignment: usize) -> MemoryResult<MemoryBlock> {
        if size == 0 {
            return Err(MemoryError::InvalidSize);
        }
        
        if !alignment.is_power_of_two() {
            return Err(MemoryError::InvalidAlignment);
        }
        
        // 计算所需的order
        let required_order = self.calculate_order(core::cmp::max(size, alignment));
        
        // 查找可用的块
        let mut found_order = None;
        for order in required_order..=self.max_order {
            if !self.free_lists[order].is_empty() {
                found_order = Some(order);
                break;
            }
        }
        
        let order = found_order.ok_or(MemoryError::OutOfMemory)?;
        
        // 取出块
        let block_index = self.free_lists[order].pop().unwrap();
        
        // 如果块太大，需要分割
        for split_order in (required_order..order).rev() {
            self.split_block(split_order + 1, block_index)?;
        }
        
        // 标记为已分配
        let block_size = self.min_block_size << required_order;
        for i in 0..(block_size / self.min_block_size) {
            self.allocated_blocks.set(block_index + i, true);
        }
        
        // 更新统计信息
        self.statistics.total_allocations += 1;
        self.statistics.current_allocated_bytes += block_size;
        if self.statistics.current_allocated_bytes > self.statistics.peak_allocated_bytes {
            self.statistics.peak_allocated_bytes = self.statistics.current_allocated_bytes;
        }
        
        let ptr = self.get_block_address(block_index);
        Ok(MemoryBlock::new(ptr, block_size, alignment))
    }
    
    fn deallocate(&mut self, block: MemoryBlock) -> MemoryResult<()> {
        if block.ptr.is_null() {
            return Err(MemoryError::NullPointer);
        }
        
        let block_index = self.get_block_index(block.ptr);
        
        // 检查是否已分配
        if !self.allocated_blocks.get(block_index) {
            return Err(MemoryError::DoubleFree);
        }
        
        // 计算order
        let order = self.calculate_order(block.size);
        let block_size = self.min_block_size << order;
        
        // 标记为未分配
        for i in 0..(block_size / self.min_block_size) {
            self.allocated_blocks.set(block_index + i, false);
        }
        
        // 尝试合并
        self.merge_blocks(order, block_index)?;
        
        // 更新统计信息
        self.statistics.total_deallocations += 1;
        self.statistics.current_allocated_bytes -= block_size;
        
        Ok(())
    }
    
    fn statistics(&self) -> MemoryStatistics {
        MemoryStatistics {
            total_size: self.memory_size,
            allocated_size: self.statistics.current_allocated_bytes,
            free_size: self.memory_size - self.statistics.current_allocated_bytes,
            allocation_count: self.statistics.total_allocations,
            deallocation_count: self.statistics.total_deallocations,
            fragmentation_ratio: self.calculate_fragmentation_ratio(),
        }
    }
    
    fn check_integrity(&self) -> MemoryResult<()> {
        // 检查空闲列表的完整性
        for (order, free_list) in self.free_lists.iter().enumerate() {
            for &block_index in free_list {
                let block_size = self.min_block_size << order;
                
                // 检查块是否在有效范围内
                if block_index * self.min_block_size + block_size > self.memory_size {
                    return Err(MemoryError::Corruption);
                }
                
                // 检查块是否真的空闲
                for i in 0..(block_size / self.min_block_size) {
                    if self.allocated_blocks.get(block_index + i) {
                        return Err(MemoryError::Corruption);
                    }
                }
            }
        }
        
        Ok(())
    }
}

impl BuddyAllocator {
    fn calculate_fragmentation_ratio(&self) -> f32 {
        if self.memory_size == 0 {
            return 0.0;
        }
        
        let mut free_blocks = 0;
        for free_list in &self.free_lists {
            free_blocks += free_list.len();
        }
        
        if free_blocks <= 1 {
            0.0
        } else {
            (free_blocks - 1) as f32 / (self.memory_size / self.min_block_size) as f32
        }
    }
    
    /// 获取详细的Buddy分配器统计信息
    pub fn get_buddy_statistics(&self) -> &BuddyStatistics {
        &self.statistics
    }
    
    /// 打印内存布局（调试用）
    pub fn print_memory_layout(&self) {
        for (order, free_list) in self.free_lists.iter().enumerate() {
            if !free_list.is_empty() {
                let block_size = self.min_block_size << order;
                println!("Order {}: {} blocks of {} bytes", order, free_list.len(), block_size);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_buddy_allocator_creation() {
        let memory = [0u8; 1024];
        let allocator = BuddyAllocator::new(
            memory.as_ptr() as *mut u8,
            1024,
            64
        );
        
        assert!(allocator.is_ok());
        let allocator = allocator.unwrap();
        assert_eq!(allocator.memory_size, 1024);
        assert_eq!(allocator.min_block_size, 64);
        assert_eq!(allocator.max_order, 4); // 1024 / 64 = 16 = 2^4
    }
    
    #[test]
    fn test_buddy_allocation() {
        let memory = [0u8; 1024];
        let mut allocator = BuddyAllocator::new(
            memory.as_ptr() as *mut u8,
            1024,
            64
        ).unwrap();
        
        // 分配一个小块
        let block1 = allocator.allocate(32);
        assert!(block1.is_ok());
        let block1 = block1.unwrap();
        assert_eq!(block1.size, 64); // 最小块大小
        
        // 分配一个大块
        let block2 = allocator.allocate(200);
        assert!(block2.is_ok());
        let block2 = block2.unwrap();
        assert_eq!(block2.size, 256); // 向上对齐到2的幂次
        
        // 释放块
        assert!(allocator.deallocate(block1).is_ok());
        assert!(allocator.deallocate(block2).is_ok());
    }
}