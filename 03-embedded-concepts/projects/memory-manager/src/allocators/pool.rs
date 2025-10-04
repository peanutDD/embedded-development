use crate::{Allocator, MemoryBlock, MemoryError, MemoryResult, MemoryStatistics, ZeroAllocAllocator, PoolStatus};
use crate::utils::{align_up, is_aligned, Bitmap};
use heapless::Vec;

/// Pool分配器
/// 
/// 预分配固定数量的固定大小块，提供确定性的内存分配
pub struct PoolAllocator {
    memory_start: *mut u8,
    memory_size: usize,
    block_size: usize,
    block_count: usize,
    allocation_bitmap: Bitmap,
    free_list: Vec<usize, 512>,
    statistics: PoolStatistics,
    initialized: bool,
}

#[derive(Debug, Clone)]
struct PoolStatistics {
    total_blocks: usize,
    allocated_blocks: usize,
    peak_allocated_blocks: usize,
    allocation_count: u32,
    deallocation_count: u32,
    allocation_failures: u32,
    fragmentation_events: u32,
}

impl PoolAllocator {
    /// 创建新的Pool分配器
    /// 
    /// # 参数
    /// - `memory_start`: 内存起始地址
    /// - `memory_size`: 内存总大小
    /// - `block_size`: 每个块的大小
    pub fn new(memory_start: *mut u8, memory_size: usize, block_size: usize) -> MemoryResult<Self> {
        if memory_start.is_null() {
            return Err(MemoryError::NullPointer);
        }
        
        if memory_size == 0 || block_size == 0 {
            return Err(MemoryError::InvalidSize);
        }
        
        // 对齐块大小
        let aligned_block_size = align_up(block_size, core::mem::align_of::<usize>());
        
        // 计算可以容纳的块数量
        let block_count = memory_size / aligned_block_size;
        
        if block_count == 0 {
            return Err(MemoryError::InvalidSize);
        }
        
        let mut allocator = Self {
            memory_start,
            memory_size,
            block_size: aligned_block_size,
            block_count,
            allocation_bitmap: Bitmap::new(block_count),
            free_list: Vec::new(),
            statistics: PoolStatistics {
                total_blocks: block_count,
                allocated_blocks: 0,
                peak_allocated_blocks: 0,
                allocation_count: 0,
                deallocation_count: 0,
                allocation_failures: 0,
                fragmentation_events: 0,
            },
            initialized: false,
        };
        
        // 初始化空闲列表
        allocator.initialize_free_list()?;
        allocator.initialized = true;
        
        Ok(allocator)
    }
    
    /// 初始化空闲列表
    fn initialize_free_list(&mut self) -> MemoryResult<()> {
        for i in 0..self.block_count {
            self.free_list.push(i).map_err(|_| MemoryError::OutOfMemory)?;
        }
        Ok(())
    }
    
    /// 获取块的地址
    fn get_block_address(&self, block_index: usize) -> *mut u8 {
        unsafe {
            self.memory_start.add(block_index * self.block_size)
        }
    }
    
    /// 从地址获取块索引
    fn get_block_index(&self, ptr: *mut u8) -> Option<usize> {
        let offset = ptr as usize - self.memory_start as usize;
        
        // 检查地址是否在有效范围内
        if offset >= self.memory_size {
            return None;
        }
        
        // 检查是否对齐到块边界
        if offset % self.block_size != 0 {
            return None;
        }
        
        let index = offset / self.block_size;
        if index < self.block_count {
            Some(index)
        } else {
            None
        }
    }
    
    /// 检查地址是否有效
    fn is_valid_address(&self, ptr: *mut u8) -> bool {
        self.get_block_index(ptr).is_some()
    }
    
    /// 分配指定数量的连续块
    fn allocate_consecutive_blocks(&mut self, block_count: usize) -> MemoryResult<usize> {
        if block_count == 0 {
            return Err(MemoryError::InvalidSize);
        }
        
        if block_count == 1 {
            // 单块分配，从空闲列表获取
            if let Some(block_index) = self.free_list.pop() {
                self.allocation_bitmap.set(block_index, true);
                self.statistics.allocated_blocks += 1;
                
                if self.statistics.allocated_blocks > self.statistics.peak_allocated_blocks {
                    self.statistics.peak_allocated_blocks = self.statistics.allocated_blocks;
                }
                
                return Ok(block_index);
            } else {
                return Err(MemoryError::OutOfMemory);
            }
        }
        
        // 多块分配，需要查找连续的空闲块
        if let Some(start_index) = self.allocation_bitmap.find_consecutive(false, block_count) {
            // 标记这些块为已分配
            self.allocation_bitmap.set_range(start_index, block_count, true);
            
            // 从空闲列表中移除这些块
            for i in start_index..start_index + block_count {
                if let Some(pos) = self.free_list.iter().position(|&x| x == i) {
                    self.free_list.remove(pos);
                }
            }
            
            self.statistics.allocated_blocks += block_count;
            
            if self.statistics.allocated_blocks > self.statistics.peak_allocated_blocks {
                self.statistics.peak_allocated_blocks = self.statistics.allocated_blocks;
            }
            
            Ok(start_index)
        } else {
            Err(MemoryError::OutOfMemory)
        }
    }
    
    /// 释放从指定索引开始的连续块
    fn deallocate_consecutive_blocks(&mut self, start_index: usize, block_count: usize) -> MemoryResult<()> {
        if start_index + block_count > self.block_count {
            return Err(MemoryError::Corruption);
        }
        
        // 检查这些块是否都已分配
        for i in start_index..start_index + block_count {
            if !self.allocation_bitmap.get(i) {
                return Err(MemoryError::DoubleFree);
            }
        }
        
        // 标记为未分配
        self.allocation_bitmap.set_range(start_index, block_count, false);
        
        // 添加到空闲列表
        for i in start_index..start_index + block_count {
            if self.free_list.push(i).is_err() {
                // 空闲列表满了，这不应该发生
                return Err(MemoryError::Corruption);
            }
        }
        
        self.statistics.allocated_blocks -= block_count;
        
        Ok(())
    }
    
    /// 计算需要的块数量
    fn calculate_required_blocks(&self, size: usize) -> usize {
        (size + self.block_size - 1) / self.block_size
    }
}

impl Allocator for PoolAllocator {
    fn allocate(&mut self, size: usize) -> MemoryResult<MemoryBlock> {
        if !self.initialized {
            return Err(MemoryError::NotInitialized);
        }
        
        if size == 0 {
            return Err(MemoryError::InvalidSize);
        }
        
        let required_blocks = self.calculate_required_blocks(size);
        
        match self.allocate_consecutive_blocks(required_blocks) {
            Ok(start_index) => {
                let ptr = self.get_block_address(start_index);
                let actual_size = required_blocks * self.block_size;
                
                self.statistics.allocation_count += 1;
                
                Ok(MemoryBlock::new(ptr, actual_size, core::mem::align_of::<usize>()))
            }
            Err(error) => {
                self.statistics.allocation_failures += 1;
                Err(error)
            }
        }
    }
    
    fn allocate_aligned(&mut self, size: usize, alignment: usize) -> MemoryResult<MemoryBlock> {
        if !alignment.is_power_of_two() {
            return Err(MemoryError::InvalidAlignment);
        }
        
        // 如果对齐要求不超过块对齐，直接分配
        if alignment <= core::mem::align_of::<usize>() {
            return self.allocate(size);
        }
        
        // 对于更严格的对齐要求，需要找到对齐的块
        let required_blocks = self.calculate_required_blocks(size);
        
        // 查找满足对齐要求的起始块
        for start_index in 0..self.block_count {
            let ptr = self.get_block_address(start_index);
            
            if is_aligned(ptr as usize, alignment) {
                // 检查从这个位置开始是否有足够的连续空闲块
                let mut consecutive_free = 0;
                for i in start_index..self.block_count {
                    if !self.allocation_bitmap.get(i) {
                        consecutive_free += 1;
                        if consecutive_free >= required_blocks {
                            break;
                        }
                    } else {
                        break;
                    }
                }
                
                if consecutive_free >= required_blocks {
                    // 分配这些块
                    self.allocation_bitmap.set_range(start_index, required_blocks, true);
                    
                    // 从空闲列表中移除
                    for i in start_index..start_index + required_blocks {
                        if let Some(pos) = self.free_list.iter().position(|&x| x == i) {
                            self.free_list.remove(pos);
                        }
                    }
                    
                    self.statistics.allocated_blocks += required_blocks;
                    self.statistics.allocation_count += 1;
                    
                    if self.statistics.allocated_blocks > self.statistics.peak_allocated_blocks {
                        self.statistics.peak_allocated_blocks = self.statistics.allocated_blocks;
                    }
                    
                    let actual_size = required_blocks * self.block_size;
                    return Ok(MemoryBlock::new(ptr, actual_size, alignment));
                }
            }
        }
        
        self.statistics.allocation_failures += 1;
        Err(MemoryError::OutOfMemory)
    }
    
    fn deallocate(&mut self, block: MemoryBlock) -> MemoryResult<()> {
        if !self.initialized {
            return Err(MemoryError::NotInitialized);
        }
        
        if block.ptr.is_null() {
            return Err(MemoryError::NullPointer);
        }
        
        let start_index = self.get_block_index(block.ptr)
            .ok_or(MemoryError::Corruption)?;
        
        let block_count = self.calculate_required_blocks(block.size);
        
        self.deallocate_consecutive_blocks(start_index, block_count)?;
        
        self.statistics.deallocation_count += 1;
        
        Ok(())
    }
    
    fn statistics(&self) -> MemoryStatistics {
        let allocated_size = self.statistics.allocated_blocks * self.block_size;
        let total_size = self.statistics.total_blocks * self.block_size;
        
        MemoryStatistics {
            total_size,
            allocated_size,
            free_size: total_size - allocated_size,
            allocation_count: self.statistics.allocation_count,
            deallocation_count: self.statistics.deallocation_count,
            fragmentation_ratio: self.calculate_fragmentation_ratio(),
        }
    }
    
    fn check_integrity(&self) -> MemoryResult<()> {
        if !self.initialized {
            return Err(MemoryError::NotInitialized);
        }
        
        // 检查分配位图和空闲列表的一致性
        let allocated_count = self.allocation_bitmap.count_ones();
        let free_count = self.allocation_bitmap.count_zeros();
        
        if allocated_count != self.statistics.allocated_blocks {
            return Err(MemoryError::Corruption);
        }
        
        if allocated_count + free_count != self.statistics.total_blocks {
            return Err(MemoryError::Corruption);
        }
        
        // 检查空闲列表中的所有块都确实是空闲的
        for &block_index in &self.free_list {
            if block_index >= self.block_count {
                return Err(MemoryError::Corruption);
            }
            
            if self.allocation_bitmap.get(block_index) {
                return Err(MemoryError::Corruption);
            }
        }
        
        Ok(())
    }
}

impl ZeroAllocAllocator for PoolAllocator {
    fn preallocate(&mut self, count: usize, size: usize) -> MemoryResult<()> {
        let required_blocks = self.calculate_required_blocks(size);
        let total_required_blocks = count * required_blocks;
        
        if total_required_blocks > self.statistics.total_blocks - self.statistics.allocated_blocks {
            return Err(MemoryError::OutOfMemory);
        }
        
        // Pool分配器本身就是预分配的，这里只是验证容量
        Ok(())
    }
    
    fn get_preallocated(&mut self, size: usize) -> MemoryResult<MemoryBlock> {
        // 对于Pool分配器，这与普通分配相同
        self.allocate(size)
    }
    
    fn return_preallocated(&mut self, block: MemoryBlock) -> MemoryResult<()> {
        // 对于Pool分配器，这与普通释放相同
        self.deallocate(block)
    }
    
    fn pool_status(&self) -> PoolStatus {
        PoolStatus {
            total_blocks: self.statistics.total_blocks,
            available_blocks: self.statistics.total_blocks - self.statistics.allocated_blocks,
            used_blocks: self.statistics.allocated_blocks,
            block_size: self.block_size,
        }
    }
}

impl PoolAllocator {
    /// 计算碎片化比率
    fn calculate_fragmentation_ratio(&self) -> f32 {
        if self.statistics.total_blocks == 0 {
            return 0.0;
        }
        
        // 计算最大连续空闲块的大小
        let mut max_consecutive_free = 0;
        let mut current_consecutive = 0;
        
        for i in 0..self.block_count {
            if !self.allocation_bitmap.get(i) {
                current_consecutive += 1;
                if current_consecutive > max_consecutive_free {
                    max_consecutive_free = current_consecutive;
                }
            } else {
                current_consecutive = 0;
            }
        }
        
        let free_blocks = self.statistics.total_blocks - self.statistics.allocated_blocks;
        
        if free_blocks == 0 {
            0.0
        } else {
            1.0 - (max_consecutive_free as f32 / free_blocks as f32)
        }
    }
    
    /// 获取详细的Pool统计信息
    pub fn get_pool_statistics(&self) -> &PoolStatistics {
        &self.statistics
    }
    
    /// 获取块大小
    pub fn block_size(&self) -> usize {
        self.block_size
    }
    
    /// 获取总块数
    pub fn block_count(&self) -> usize {
        self.block_count
    }
    
    /// 获取利用率
    pub fn utilization(&self) -> f32 {
        if self.statistics.total_blocks == 0 {
            0.0
        } else {
            self.statistics.allocated_blocks as f32 / self.statistics.total_blocks as f32
        }
    }
    
    /// 获取分配成功率
    pub fn allocation_success_rate(&self) -> f32 {
        let total_attempts = self.statistics.allocation_count + self.statistics.allocation_failures;
        if total_attempts == 0 {
            1.0
        } else {
            self.statistics.allocation_count as f32 / total_attempts as f32
        }
    }
    
    /// 重置统计信息
    pub fn reset_statistics(&mut self) {
        self.statistics.allocation_count = 0;
        self.statistics.deallocation_count = 0;
        self.statistics.allocation_failures = 0;
        self.statistics.fragmentation_events = 0;
        // 保留当前分配状态相关的统计
    }
    
    /// 压缩内存池（重新整理空闲列表）
    pub fn compact(&mut self) -> MemoryResult<()> {
        // 重建空闲列表，按索引顺序排列
        self.free_list.clear();
        
        for i in 0..self.block_count {
            if !self.allocation_bitmap.get(i) {
                if self.free_list.push(i).is_err() {
                    return Err(MemoryError::OutOfMemory);
                }
            }
        }
        
        Ok(())
    }
    
    /// 查找最大连续空闲块
    pub fn find_largest_free_block(&self) -> usize {
        let mut max_consecutive = 0;
        let mut current_consecutive = 0;
        
        for i in 0..self.block_count {
            if !self.allocation_bitmap.get(i) {
                current_consecutive += 1;
                if current_consecutive > max_consecutive {
                    max_consecutive = current_consecutive;
                }
            } else {
                current_consecutive = 0;
            }
        }
        
        max_consecutive * self.block_size
    }
    
    /// 打印内存池状态（调试用）
    pub fn print_pool_status(&self) {
        let status = self.pool_status();
        
        println!("Pool Status:");
        println!("  Total blocks: {}", status.total_blocks);
        println!("  Used blocks: {}", status.used_blocks);
        println!("  Available blocks: {}", status.available_blocks);
        println!("  Block size: {} bytes", status.block_size);
        println!("  Utilization: {:.1}%", status.utilization() * 100.0);
        println!("  Allocation success rate: {:.1}%", self.allocation_success_rate() * 100.0);
        println!("  Fragmentation ratio: {:.1}%", self.calculate_fragmentation_ratio() * 100.0);
        println!("  Largest free block: {} bytes", self.find_largest_free_block());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_pool_allocator_creation() {
        let memory = [0u8; 4096];
        let allocator = PoolAllocator::new(
            memory.as_ptr() as *mut u8,
            4096,
            64
        );
        
        assert!(allocator.is_ok());
        let allocator = allocator.unwrap();
        assert_eq!(allocator.block_size(), 64);
        assert_eq!(allocator.block_count(), 64); // 4096 / 64 = 64
    }
    
    #[test]
    fn test_pool_allocation() {
        let memory = [0u8; 4096];
        let mut allocator = PoolAllocator::new(
            memory.as_ptr() as *mut u8,
            4096,
            128
        ).unwrap();
        
        // 分配一个块
        let block1 = allocator.allocate(100);
        assert!(block1.is_ok());
        let block1 = block1.unwrap();
        assert_eq!(block1.size, 128); // 向上对齐到块大小
        
        // 分配另一个块
        let block2 = allocator.allocate(128);
        assert!(block2.is_ok());
        
        // 释放块
        assert!(allocator.deallocate(block1).is_ok());
        assert!(allocator.deallocate(block2.unwrap()).is_ok());
    }
    
    #[test]
    fn test_pool_large_allocation() {
        let memory = [0u8; 4096];
        let mut allocator = PoolAllocator::new(
            memory.as_ptr() as *mut u8,
            4096,
            64
        ).unwrap();
        
        // 分配需要多个块的大内存
        let block = allocator.allocate(200); // 需要4个64字节的块
        assert!(block.is_ok());
        let block = block.unwrap();
        assert_eq!(block.size, 256); // 4 * 64 = 256
        
        assert!(allocator.deallocate(block).is_ok());
    }
    
    #[test]
    fn test_pool_exhaustion() {
        let memory = [0u8; 512];
        let mut allocator = PoolAllocator::new(
            memory.as_ptr() as *mut u8,
            512,
            64
        ).unwrap();
        
        let mut blocks = Vec::<MemoryBlock, 16>::new();
        
        // 分配所有可用的块
        for _ in 0..8 { // 512 / 64 = 8 blocks
            if let Ok(block) = allocator.allocate(64) {
                let _ = blocks.push(block);
            }
        }
        
        // 尝试再分配一个块应该失败
        assert_eq!(allocator.allocate(64), Err(MemoryError::OutOfMemory));
        
        // 释放所有块
        for block in blocks {
            assert!(allocator.deallocate(block).is_ok());
        }
    }
    
    #[test]
    fn test_pool_integrity() {
        let memory = [0u8; 2048];
        let mut allocator = PoolAllocator::new(
            memory.as_ptr() as *mut u8,
            2048,
            128
        ).unwrap();
        
        // 初始完整性检查
        assert!(allocator.check_integrity().is_ok());
        
        // 分配一些块
        let block1 = allocator.allocate(128).unwrap();
        let block2 = allocator.allocate(256).unwrap(); // 需要2个块
        
        // 检查完整性
        assert!(allocator.check_integrity().is_ok());
        
        // 释放块
        assert!(allocator.deallocate(block1).is_ok());
        assert!(allocator.deallocate(block2).is_ok());
        
        // 最终完整性检查
        assert!(allocator.check_integrity().is_ok());
    }
    
    #[test]
    fn test_zero_alloc_interface() {
        let memory = [0u8; 4096];
        let mut allocator = PoolAllocator::new(
            memory.as_ptr() as *mut u8,
            4096,
            64
        ).unwrap();
        
        // 测试预分配
        assert!(allocator.preallocate(10, 64).is_ok());
        
        // 测试获取预分配的内存
        let block = allocator.get_preallocated(64);
        assert!(block.is_ok());
        
        // 测试返回预分配的内存
        assert!(allocator.return_preallocated(block.unwrap()).is_ok());
        
        // 测试池状态
        let status = allocator.pool_status();
        assert_eq!(status.block_size, 64);
        assert!(status.total_blocks > 0);
    }
}