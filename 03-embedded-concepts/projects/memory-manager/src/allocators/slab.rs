use crate::{Allocator, MemoryBlock, MemoryError, MemoryResult, MemoryStatistics};
use crate::utils::{align_up, is_aligned};
use heapless::Vec;

/// Slab分配器
/// 
/// 专门用于分配固定大小的对象，提供O(1)时间复杂度的分配和释放
pub struct SlabAllocator {
    memory_start: *mut u8,
    memory_size: usize,
    object_size: usize,
    objects_per_slab: usize,
    slabs: Vec<Slab, 16>,
    free_objects: Vec<*mut u8, 256>,
    statistics: SlabStatistics,
}

/// 单个Slab
struct Slab {
    start_address: *mut u8,
    free_count: usize,
    total_count: usize,
    free_list: Vec<*mut u8, 64>,
}

#[derive(Debug, Clone)]
struct SlabStatistics {
    total_slabs: usize,
    active_slabs: usize,
    total_objects: usize,
    allocated_objects: usize,
    allocation_count: u32,
    deallocation_count: u32,
    slab_allocations: u32,
    cache_hits: u32,
    cache_misses: u32,
}

impl SlabAllocator {
    /// 创建新的Slab分配器
    /// 
    /// # 参数
    /// - `memory_start`: 内存起始地址
    /// - `memory_size`: 内存总大小
    /// - `object_size`: 对象大小
    pub fn new(memory_start: *mut u8, memory_size: usize, object_size: usize) -> MemoryResult<Self> {
        if object_size == 0 || memory_size == 0 {
            return Err(MemoryError::InvalidSize);
        }
        
        if memory_start.is_null() {
            return Err(MemoryError::NullPointer);
        }
        
        // 对象大小至少要能容纳一个指针（用于空闲链表）
        let aligned_object_size = align_up(
            core::cmp::max(object_size, core::mem::size_of::<*mut u8>()),
            core::mem::align_of::<*mut u8>()
        );
        
        // 计算每个slab能容纳的对象数量
        let slab_size = 4096; // 4KB per slab
        let objects_per_slab = slab_size / aligned_object_size;
        
        if objects_per_slab == 0 {
            return Err(MemoryError::InvalidSize);
        }
        
        let mut allocator = Self {
            memory_start,
            memory_size,
            object_size: aligned_object_size,
            objects_per_slab,
            slabs: Vec::new(),
            free_objects: Vec::new(),
            statistics: SlabStatistics {
                total_slabs: 0,
                active_slabs: 0,
                total_objects: 0,
                allocated_objects: 0,
                allocation_count: 0,
                deallocation_count: 0,
                slab_allocations: 0,
                cache_hits: 0,
                cache_misses: 0,
            },
        };
        
        // 创建初始slab
        allocator.allocate_new_slab()?;
        
        Ok(allocator)
    }
    
    /// 分配新的slab
    fn allocate_new_slab(&mut self) -> MemoryResult<()> {
        let slab_size = self.objects_per_slab * self.object_size;
        
        // 检查是否有足够的内存
        let used_memory = self.slabs.len() * slab_size;
        if used_memory + slab_size > self.memory_size {
            return Err(MemoryError::OutOfMemory);
        }
        
        // 计算新slab的起始地址
        let slab_start = unsafe {
            self.memory_start.add(used_memory)
        };
        
        let mut slab = Slab {
            start_address: slab_start,
            free_count: self.objects_per_slab,
            total_count: self.objects_per_slab,
            free_list: Vec::new(),
        };
        
        // 初始化空闲对象链表
        for i in 0..self.objects_per_slab {
            let object_ptr = unsafe {
                slab_start.add(i * self.object_size)
            };
            
            if slab.free_list.push(object_ptr).is_err() {
                return Err(MemoryError::OutOfMemory);
            }
        }
        
        // 添加到slab列表
        self.slabs.push(slab).map_err(|_| MemoryError::OutOfMemory)?;
        
        // 更新统计信息
        self.statistics.total_slabs += 1;
        self.statistics.active_slabs += 1;
        self.statistics.total_objects += self.objects_per_slab;
        self.statistics.slab_allocations += 1;
        
        Ok(())
    }
    
    /// 查找包含指定地址的slab
    fn find_slab_for_address(&self, ptr: *mut u8) -> Option<usize> {
        for (index, slab) in self.slabs.iter().enumerate() {
            let slab_end = unsafe {
                slab.start_address.add(self.objects_per_slab * self.object_size)
            };
            
            if ptr >= slab.start_address && ptr < slab_end {
                return Some(index);
            }
        }
        None
    }
    
    /// 验证地址是否为有效的对象地址
    fn is_valid_object_address(&self, ptr: *mut u8) -> bool {
        if let Some(slab_index) = self.find_slab_for_address(ptr) {
            let slab = &self.slabs[slab_index];
            let offset = ptr as usize - slab.start_address as usize;
            
            // 检查是否对齐到对象边界
            offset % self.object_size == 0
        } else {
            false
        }
    }
    
    /// 获取对象在slab中的索引
    fn get_object_index(&self, slab_index: usize, ptr: *mut u8) -> usize {
        let slab = &self.slabs[slab_index];
        let offset = ptr as usize - slab.start_address as usize;
        offset / self.object_size
    }
}

impl Allocator for SlabAllocator {
    fn allocate(&mut self, size: usize) -> MemoryResult<MemoryBlock> {
        if size > self.object_size {
            return Err(MemoryError::InvalidSize);
        }
        
        // 首先尝试从全局空闲列表分配
        if let Some(ptr) = self.free_objects.pop() {
            self.statistics.cache_hits += 1;
            self.statistics.allocation_count += 1;
            self.statistics.allocated_objects += 1;
            
            return Ok(MemoryBlock::new(ptr, self.object_size, core::mem::align_of::<*mut u8>()));
        }
        
        self.statistics.cache_misses += 1;
        
        // 查找有空闲对象的slab
        for slab in &mut self.slabs {
            if slab.free_count > 0 {
                if let Some(ptr) = slab.free_list.pop() {
                    slab.free_count -= 1;
                    
                    self.statistics.allocation_count += 1;
                    self.statistics.allocated_objects += 1;
                    
                    return Ok(MemoryBlock::new(ptr, self.object_size, core::mem::align_of::<*mut u8>()));
                }
            }
        }
        
        // 没有空闲对象，分配新的slab
        self.allocate_new_slab()?;
        
        // 从新slab分配
        let last_slab = self.slabs.last_mut().unwrap();
        if let Some(ptr) = last_slab.free_list.pop() {
            last_slab.free_count -= 1;
            
            self.statistics.allocation_count += 1;
            self.statistics.allocated_objects += 1;
            
            Ok(MemoryBlock::new(ptr, self.object_size, core::mem::align_of::<*mut u8>()))
        } else {
            Err(MemoryError::OutOfMemory)
        }
    }
    
    fn allocate_aligned(&mut self, size: usize, alignment: usize) -> MemoryResult<MemoryBlock> {
        if alignment > core::mem::align_of::<*mut u8>() {
            return Err(MemoryError::InvalidAlignment);
        }
        
        self.allocate(size)
    }
    
    fn deallocate(&mut self, block: MemoryBlock) -> MemoryResult<()> {
        if block.ptr.is_null() {
            return Err(MemoryError::NullPointer);
        }
        
        if block.size != self.object_size {
            return Err(MemoryError::InvalidSize);
        }
        
        // 验证地址有效性
        if !self.is_valid_object_address(block.ptr) {
            return Err(MemoryError::Corruption);
        }
        
        // 查找对应的slab
        let slab_index = self.find_slab_for_address(block.ptr)
            .ok_or(MemoryError::Corruption)?;
        
        let slab = &mut self.slabs[slab_index];
        
        // 检查是否重复释放
        if slab.free_list.iter().any(|&ptr| ptr == block.ptr) {
            return Err(MemoryError::DoubleFree);
        }
        
        // 将对象添加到slab的空闲列表
        slab.free_list.push(block.ptr).map_err(|_| MemoryError::OutOfMemory)?;
        slab.free_count += 1;
        
        // 更新统计信息
        self.statistics.deallocation_count += 1;
        self.statistics.allocated_objects -= 1;
        
        // 如果slab完全空闲且不是最后一个slab，考虑释放它
        if slab.free_count == slab.total_count && self.slabs.len() > 1 {
            self.try_free_empty_slab(slab_index);
        }
        
        Ok(())
    }
    
    fn statistics(&self) -> MemoryStatistics {
        let allocated_size = self.statistics.allocated_objects * self.object_size;
        let total_size = self.statistics.total_objects * self.object_size;
        
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
        let mut total_free_objects = 0;
        
        // 检查每个slab的完整性
        for (index, slab) in self.slabs.iter().enumerate() {
            // 检查空闲计数
            if slab.free_count != slab.free_list.len() {
                return Err(MemoryError::Corruption);
            }
            
            // 检查空闲列表中的地址
            for &ptr in &slab.free_list {
                if !self.is_valid_object_address(ptr) {
                    return Err(MemoryError::Corruption);
                }
                
                if self.find_slab_for_address(ptr) != Some(index) {
                    return Err(MemoryError::Corruption);
                }
            }
            
            total_free_objects += slab.free_count;
        }
        
        // 检查统计信息一致性
        let expected_allocated = self.statistics.total_objects - total_free_objects;
        if expected_allocated != self.statistics.allocated_objects {
            return Err(MemoryError::Corruption);
        }
        
        Ok(())
    }
}

impl SlabAllocator {
    /// 尝试释放空的slab
    fn try_free_empty_slab(&mut self, slab_index: usize) {
        if slab_index < self.slabs.len() && self.slabs.len() > 1 {
            let slab = &self.slabs[slab_index];
            
            if slab.free_count == slab.total_count {
                // 移除空的slab
                self.slabs.remove(slab_index);
                
                // 更新统计信息
                self.statistics.active_slabs -= 1;
                self.statistics.total_objects -= self.objects_per_slab;
            }
        }
    }
    
    /// 计算碎片化比率
    fn calculate_fragmentation_ratio(&self) -> f32 {
        if self.statistics.total_objects == 0 {
            return 0.0;
        }
        
        let mut partially_used_slabs = 0;
        
        for slab in &self.slabs {
            if slab.free_count > 0 && slab.free_count < slab.total_count {
                partially_used_slabs += 1;
            }
        }
        
        if self.slabs.len() == 0 {
            0.0
        } else {
            partially_used_slabs as f32 / self.slabs.len() as f32
        }
    }
    
    /// 获取详细的Slab统计信息
    pub fn get_slab_statistics(&self) -> &SlabStatistics {
        &self.statistics
    }
    
    /// 获取对象大小
    pub fn object_size(&self) -> usize {
        self.object_size
    }
    
    /// 获取每个slab的对象数量
    pub fn objects_per_slab(&self) -> usize {
        self.objects_per_slab
    }
    
    /// 获取缓存命中率
    pub fn cache_hit_rate(&self) -> f32 {
        let total_requests = self.statistics.cache_hits + self.statistics.cache_misses;
        if total_requests == 0 {
            0.0
        } else {
            self.statistics.cache_hits as f32 / total_requests as f32
        }
    }
    
    /// 预分配指定数量的对象
    pub fn preallocate(&mut self, count: usize) -> MemoryResult<()> {
        let needed_slabs = (count + self.objects_per_slab - 1) / self.objects_per_slab;
        let current_slabs = self.slabs.len();
        
        for _ in current_slabs..needed_slabs {
            self.allocate_new_slab()?;
        }
        
        Ok(())
    }
    
    /// 压缩内存，释放空的slab
    pub fn compact(&mut self) -> usize {
        let initial_slabs = self.slabs.len();
        
        // 从后往前检查，释放空的slab
        let mut i = self.slabs.len();
        while i > 1 {
            i -= 1;
            if self.slabs[i].free_count == self.slabs[i].total_count {
                self.slabs.remove(i);
                self.statistics.active_slabs -= 1;
                self.statistics.total_objects -= self.objects_per_slab;
            }
        }
        
        initial_slabs - self.slabs.len()
    }
    
    /// 打印slab使用情况（调试用）
    pub fn print_slab_usage(&self) {
        for (i, slab) in self.slabs.iter().enumerate() {
            let used = slab.total_count - slab.free_count;
            let usage_percent = (used as f32 / slab.total_count as f32) * 100.0;
            
            println!("Slab {}: {}/{} objects used ({:.1}%)", 
                     i, used, slab.total_count, usage_percent);
        }
        
        println!("Cache hit rate: {:.1}%", self.cache_hit_rate() * 100.0);
        println!("Fragmentation ratio: {:.1}%", self.calculate_fragmentation_ratio() * 100.0);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_slab_allocator_creation() {
        let memory = [0u8; 8192];
        let allocator = SlabAllocator::new(
            memory.as_ptr() as *mut u8,
            8192,
            64
        );
        
        assert!(allocator.is_ok());
        let allocator = allocator.unwrap();
        assert_eq!(allocator.object_size(), 64);
        assert!(allocator.objects_per_slab() > 0);
    }
    
    #[test]
    fn test_slab_allocation() {
        let memory = [0u8; 8192];
        let mut allocator = SlabAllocator::new(
            memory.as_ptr() as *mut u8,
            8192,
            32
        ).unwrap();
        
        // 分配一些对象
        let block1 = allocator.allocate(32);
        assert!(block1.is_ok());
        let block1 = block1.unwrap();
        assert_eq!(block1.size, 32);
        
        let block2 = allocator.allocate(16); // 小于对象大小也应该成功
        assert!(block2.is_ok());
        
        // 释放对象
        assert!(allocator.deallocate(block1).is_ok());
        assert!(allocator.deallocate(block2.unwrap()).is_ok());
    }
    
    #[test]
    fn test_slab_reuse() {
        let memory = [0u8; 8192];
        let mut allocator = SlabAllocator::new(
            memory.as_ptr() as *mut u8,
            8192,
            64
        ).unwrap();
        
        // 分配并释放一个对象
        let block1 = allocator.allocate(64).unwrap();
        let ptr1 = block1.ptr;
        allocator.deallocate(block1).unwrap();
        
        // 再次分配应该重用相同的地址
        let block2 = allocator.allocate(64).unwrap();
        assert_eq!(block2.ptr, ptr1);
        
        allocator.deallocate(block2).unwrap();
    }
    
    #[test]
    fn test_slab_integrity() {
        let memory = [0u8; 8192];
        let mut allocator = SlabAllocator::new(
            memory.as_ptr() as *mut u8,
            8192,
            128
        ).unwrap();
        
        // 分配多个对象
        let mut blocks = Vec::<MemoryBlock, 32>::new();
        for _ in 0..10 {
            if let Ok(block) = allocator.allocate(128) {
                let _ = blocks.push(block);
            }
        }
        
        // 检查完整性
        assert!(allocator.check_integrity().is_ok());
        
        // 释放所有对象
        for block in blocks {
            assert!(allocator.deallocate(block).is_ok());
        }
        
        // 再次检查完整性
        assert!(allocator.check_integrity().is_ok());
    }
    
    #[test]
    fn test_double_free_detection() {
        let memory = [0u8; 4096];
        let mut allocator = SlabAllocator::new(
            memory.as_ptr() as *mut u8,
            4096,
            64
        ).unwrap();
        
        let block = allocator.allocate(64).unwrap();
        
        // 第一次释放应该成功
        assert!(allocator.deallocate(block).is_ok());
        
        // 第二次释放应该失败
        assert_eq!(allocator.deallocate(block), Err(MemoryError::DoubleFree));
    }
}