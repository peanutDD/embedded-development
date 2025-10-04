use crate::{Allocator, MemoryBlock, MemoryError, MemoryResult, MemoryStatistics};
use crate::utils::{align_up, is_aligned};
use heapless::Vec;

/// Stack分配器
/// 
/// 按栈的方式管理内存，只能按LIFO顺序释放，提供极快的分配速度
pub struct StackAllocator {
    memory_start: *mut u8,
    memory_size: usize,
    current_top: *mut u8,
    stack_frames: Vec<StackFrame, 32>,
    statistics: StackStatistics,
    alignment: usize,
}

/// 栈帧信息
#[derive(Debug, Clone, Copy)]
struct StackFrame {
    start_address: *mut u8,
    size: usize,
    alignment: usize,
    frame_id: u32,
}

#[derive(Debug, Clone)]
struct StackStatistics {
    total_allocations: u32,
    total_deallocations: u32,
    current_allocated_bytes: usize,
    peak_allocated_bytes: usize,
    stack_depth: usize,
    max_stack_depth: usize,
    alignment_waste: usize,
    out_of_order_deallocations: u32,
}

impl StackAllocator {
    /// 创建新的Stack分配器
    /// 
    /// # 参数
    /// - `memory_start`: 内存起始地址
    /// - `memory_size`: 内存总大小
    /// - `default_alignment`: 默认对齐要求
    pub fn new(memory_start: *mut u8, memory_size: usize, default_alignment: usize) -> MemoryResult<Self> {
        if memory_start.is_null() {
            return Err(MemoryError::NullPointer);
        }
        
        if memory_size == 0 {
            return Err(MemoryError::InvalidSize);
        }
        
        if !default_alignment.is_power_of_two() {
            return Err(MemoryError::InvalidAlignment);
        }
        
        Ok(Self {
            memory_start,
            memory_size,
            current_top: memory_start,
            stack_frames: Vec::new(),
            statistics: StackStatistics {
                total_allocations: 0,
                total_deallocations: 0,
                current_allocated_bytes: 0,
                peak_allocated_bytes: 0,
                stack_depth: 0,
                max_stack_depth: 0,
                alignment_waste: 0,
                out_of_order_deallocations: 0,
            },
            alignment: default_alignment,
        })
    }
    
    /// 获取当前已使用的内存大小
    fn get_used_size(&self) -> usize {
        self.current_top as usize - self.memory_start as usize
    }
    
    /// 获取剩余可用内存大小
    fn get_available_size(&self) -> usize {
        self.memory_size - self.get_used_size()
    }
    
    /// 检查地址是否在栈的有效范围内
    fn is_valid_address(&self, ptr: *mut u8) -> bool {
        ptr >= self.memory_start && ptr < unsafe { self.memory_start.add(self.memory_size) }
    }
    
    /// 查找包含指定地址的栈帧
    fn find_frame_for_address(&self, ptr: *mut u8) -> Option<usize> {
        for (index, frame) in self.stack_frames.iter().enumerate() {
            let frame_end = unsafe { frame.start_address.add(frame.size) };
            if ptr >= frame.start_address && ptr < frame_end {
                return Some(index);
            }
        }
        None
    }
    
    /// 生成新的帧ID
    fn generate_frame_id(&self) -> u32 {
        static mut NEXT_FRAME_ID: u32 = 1;
        unsafe {
            let id = NEXT_FRAME_ID;
            NEXT_FRAME_ID = NEXT_FRAME_ID.wrapping_add(1);
            id
        }
    }
    
    /// 创建栈帧标记
    pub fn create_marker(&self) -> StackMarker {
        StackMarker {
            top_address: self.current_top,
            frame_count: self.stack_frames.len(),
        }
    }
    
    /// 恢复到指定标记
    pub fn restore_to_marker(&mut self, marker: StackMarker) -> MemoryResult<()> {
        // 验证标记的有效性
        if marker.top_address < self.memory_start || 
           marker.top_address > unsafe { self.memory_start.add(self.memory_size) } {
            return Err(MemoryError::Corruption);
        }
        
        if marker.frame_count > self.stack_frames.len() {
            return Err(MemoryError::Corruption);
        }
        
        // 计算需要释放的内存大小
        let released_size = self.current_top as usize - marker.top_address as usize;
        
        // 恢复栈顶
        self.current_top = marker.top_address;
        
        // 移除多余的栈帧
        while self.stack_frames.len() > marker.frame_count {
            self.stack_frames.pop();
        }
        
        // 更新统计信息
        self.statistics.current_allocated_bytes -= released_size;
        self.statistics.stack_depth = self.stack_frames.len();
        
        Ok(())
    }
    
    /// 清空整个栈
    pub fn clear(&mut self) {
        self.current_top = self.memory_start;
        self.stack_frames.clear();
        
        // 更新统计信息
        self.statistics.current_allocated_bytes = 0;
        self.statistics.stack_depth = 0;
    }
    
    /// 获取栈的使用情况
    pub fn get_stack_usage(&self) -> StackUsage {
        StackUsage {
            total_size: self.memory_size,
            used_size: self.get_used_size(),
            available_size: self.get_available_size(),
            frame_count: self.stack_frames.len(),
            utilization: self.get_used_size() as f32 / self.memory_size as f32,
        }
    }
}

/// 栈标记，用于批量释放
#[derive(Debug, Clone, Copy)]
pub struct StackMarker {
    top_address: *mut u8,
    frame_count: usize,
}

/// 栈使用情况
#[derive(Debug, Clone, Copy)]
pub struct StackUsage {
    pub total_size: usize,
    pub used_size: usize,
    pub available_size: usize,
    pub frame_count: usize,
    pub utilization: f32,
}

impl Allocator for StackAllocator {
    fn allocate(&mut self, size: usize) -> MemoryResult<MemoryBlock> {
        self.allocate_aligned(size, self.alignment)
    }
    
    fn allocate_aligned(&mut self, size: usize, alignment: usize) -> MemoryResult<MemoryBlock> {
        if size == 0 {
            return Err(MemoryError::InvalidSize);
        }
        
        if !alignment.is_power_of_two() {
            return Err(MemoryError::InvalidAlignment);
        }
        
        // 计算对齐后的起始地址
        let aligned_start = align_up(self.current_top as usize, alignment);
        let aligned_ptr = aligned_start as *mut u8;
        
        // 检查是否有足够的内存
        let end_address = aligned_start + size;
        let memory_end = self.memory_start as usize + self.memory_size;
        
        if end_address > memory_end {
            return Err(MemoryError::OutOfMemory);
        }
        
        // 计算对齐浪费的字节数
        let alignment_waste = aligned_start - self.current_top as usize;
        
        // 创建栈帧
        let frame = StackFrame {
            start_address: aligned_ptr,
            size,
            alignment,
            frame_id: self.generate_frame_id(),
        };
        
        // 添加到栈帧列表
        self.stack_frames.push(frame).map_err(|_| MemoryError::OutOfMemory)?;
        
        // 更新栈顶
        self.current_top = unsafe { aligned_ptr.add(size) };
        
        // 更新统计信息
        self.statistics.total_allocations += 1;
        self.statistics.current_allocated_bytes += size;
        self.statistics.alignment_waste += alignment_waste;
        self.statistics.stack_depth = self.stack_frames.len();
        
        if self.statistics.current_allocated_bytes > self.statistics.peak_allocated_bytes {
            self.statistics.peak_allocated_bytes = self.statistics.current_allocated_bytes;
        }
        
        if self.statistics.stack_depth > self.statistics.max_stack_depth {
            self.statistics.max_stack_depth = self.statistics.stack_depth;
        }
        
        Ok(MemoryBlock::new(aligned_ptr, size, alignment))
    }
    
    fn deallocate(&mut self, block: MemoryBlock) -> MemoryResult<()> {
        if block.ptr.is_null() {
            return Err(MemoryError::NullPointer);
        }
        
        if !self.is_valid_address(block.ptr) {
            return Err(MemoryError::Corruption);
        }
        
        // 查找对应的栈帧
        let frame_index = self.find_frame_for_address(block.ptr)
            .ok_or(MemoryError::Corruption)?;
        
        let frame = self.stack_frames[frame_index];
        
        // 验证块信息
        if frame.start_address != block.ptr || frame.size != block.size {
            return Err(MemoryError::Corruption);
        }
        
        // 检查是否按LIFO顺序释放
        if frame_index != self.stack_frames.len() - 1 {
            self.statistics.out_of_order_deallocations += 1;
            return Err(MemoryError::InvalidSize); // 使用InvalidSize表示顺序错误
        }
        
        // 移除栈帧
        self.stack_frames.pop();
        
        // 更新栈顶
        self.current_top = frame.start_address;
        
        // 更新统计信息
        self.statistics.total_deallocations += 1;
        self.statistics.current_allocated_bytes -= frame.size;
        self.statistics.stack_depth = self.stack_frames.len();
        
        Ok(())
    }
    
    fn statistics(&self) -> MemoryStatistics {
        MemoryStatistics {
            total_size: self.memory_size,
            allocated_size: self.statistics.current_allocated_bytes,
            free_size: self.get_available_size(),
            allocation_count: self.statistics.total_allocations,
            deallocation_count: self.statistics.total_deallocations,
            fragmentation_ratio: 0.0, // Stack分配器没有碎片
        }
    }
    
    fn check_integrity(&self) -> MemoryResult<()> {
        // 检查栈顶是否在有效范围内
        if !self.is_valid_address(self.current_top) {
            return Err(MemoryError::Corruption);
        }
        
        // 检查栈帧的连续性
        let mut expected_top = self.memory_start;
        
        for frame in &self.stack_frames {
            // 检查帧地址是否有效
            if !self.is_valid_address(frame.start_address) {
                return Err(MemoryError::Corruption);
            }
            
            // 检查帧是否在预期位置之后
            if frame.start_address < expected_top {
                return Err(MemoryError::Corruption);
            }
            
            // 检查对齐
            if !is_aligned(frame.start_address as usize, frame.alignment) {
                return Err(MemoryError::Corruption);
            }
            
            // 更新预期的下一个位置
            expected_top = unsafe { frame.start_address.add(frame.size) };
        }
        
        // 检查栈顶是否与最后一个帧的结束位置匹配
        if expected_top != self.current_top {
            return Err(MemoryError::Corruption);
        }
        
        // 检查统计信息的一致性
        let calculated_allocated = self.get_used_size();
        if calculated_allocated != self.statistics.current_allocated_bytes {
            return Err(MemoryError::Corruption);
        }
        
        Ok(())
    }
}

impl StackAllocator {
    /// 获取详细的Stack统计信息
    pub fn get_stack_statistics(&self) -> &StackStatistics {
        &self.statistics
    }
    
    /// 获取对齐效率
    pub fn alignment_efficiency(&self) -> f32 {
        if self.statistics.current_allocated_bytes == 0 {
            1.0
        } else {
            let total_requested = self.statistics.current_allocated_bytes + self.statistics.alignment_waste;
            self.statistics.current_allocated_bytes as f32 / total_requested as f32
        }
    }
    
    /// 获取栈深度
    pub fn stack_depth(&self) -> usize {
        self.stack_frames.len()
    }
    
    /// 获取最大栈深度
    pub fn max_stack_depth(&self) -> usize {
        self.statistics.max_stack_depth
    }
    
    /// 检查是否为空栈
    pub fn is_empty(&self) -> bool {
        self.stack_frames.is_empty()
    }
    
    /// 获取顶部栈帧信息
    pub fn top_frame(&self) -> Option<StackFrameInfo> {
        self.stack_frames.last().map(|frame| StackFrameInfo {
            start_address: frame.start_address,
            size: frame.size,
            alignment: frame.alignment,
            frame_id: frame.frame_id,
        })
    }
    
    /// 获取所有栈帧信息
    pub fn get_all_frames(&self) -> Vec<StackFrameInfo, 32> {
        let mut frames = Vec::new();
        
        for frame in &self.stack_frames {
            let frame_info = StackFrameInfo {
                start_address: frame.start_address,
                size: frame.size,
                alignment: frame.alignment,
                frame_id: frame.frame_id,
            };
            
            if frames.push(frame_info).is_err() {
                break;
            }
        }
        
        frames
    }
    
    /// 重置统计信息
    pub fn reset_statistics(&mut self) {
        self.statistics.total_allocations = 0;
        self.statistics.total_deallocations = 0;
        self.statistics.peak_allocated_bytes = self.statistics.current_allocated_bytes;
        self.statistics.max_stack_depth = self.statistics.stack_depth;
        self.statistics.alignment_waste = 0;
        self.statistics.out_of_order_deallocations = 0;
    }
    
    /// 打印栈状态（调试用）
    pub fn print_stack_status(&self) {
        let usage = self.get_stack_usage();
        
        println!("Stack Allocator Status:");
        println!("  Total size: {} bytes", usage.total_size);
        println!("  Used size: {} bytes", usage.used_size);
        println!("  Available size: {} bytes", usage.available_size);
        println!("  Utilization: {:.1}%", usage.utilization * 100.0);
        println!("  Frame count: {}", usage.frame_count);
        println!("  Max depth: {}", self.statistics.max_stack_depth);
        println!("  Alignment efficiency: {:.1}%", self.alignment_efficiency() * 100.0);
        println!("  Out-of-order deallocations: {}", self.statistics.out_of_order_deallocations);
        
        if !self.stack_frames.is_empty() {
            println!("  Stack frames:");
            for (i, frame) in self.stack_frames.iter().enumerate() {
                println!("    Frame {}: {:p}, {} bytes, align {}", 
                         i, frame.start_address, frame.size, frame.alignment);
            }
        }
    }
}

/// 栈帧信息（用于外部查询）
#[derive(Debug, Clone, Copy)]
pub struct StackFrameInfo {
    pub start_address: *mut u8,
    pub size: usize,
    pub alignment: usize,
    pub frame_id: u32,
}

/// 栈分配器的RAII包装器
pub struct StackScope<'a> {
    allocator: &'a mut StackAllocator,
    marker: StackMarker,
}

impl<'a> StackScope<'a> {
    /// 创建新的栈作用域
    pub fn new(allocator: &'a mut StackAllocator) -> Self {
        let marker = allocator.create_marker();
        Self { allocator, marker }
    }
    
    /// 在作用域内分配内存
    pub fn allocate(&mut self, size: usize) -> MemoryResult<MemoryBlock> {
        self.allocator.allocate(size)
    }
    
    /// 在作用域内分配对齐内存
    pub fn allocate_aligned(&mut self, size: usize, alignment: usize) -> MemoryResult<MemoryBlock> {
        self.allocator.allocate_aligned(size, alignment)
    }
}

impl<'a> Drop for StackScope<'a> {
    fn drop(&mut self) {
        // 自动恢复到作用域开始时的状态
        let _ = self.allocator.restore_to_marker(self.marker);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_stack_allocator_creation() {
        let memory = [0u8; 4096];
        let allocator = StackAllocator::new(
            memory.as_ptr() as *mut u8,
            4096,
            8
        );
        
        assert!(allocator.is_ok());
        let allocator = allocator.unwrap();
        assert_eq!(allocator.memory_size, 4096);
        assert!(allocator.is_empty());
    }
    
    #[test]
    fn test_stack_allocation() {
        let memory = [0u8; 4096];
        let mut allocator = StackAllocator::new(
            memory.as_ptr() as *mut u8,
            4096,
            8
        ).unwrap();
        
        // 分配一些内存
        let block1 = allocator.allocate(100);
        assert!(block1.is_ok());
        let block1 = block1.unwrap();
        
        let block2 = allocator.allocate(200);
        assert!(block2.is_ok());
        let block2 = block2.unwrap();
        
        assert_eq!(allocator.stack_depth(), 2);
        
        // 按LIFO顺序释放
        assert!(allocator.deallocate(block2).is_ok());
        assert_eq!(allocator.stack_depth(), 1);
        
        assert!(allocator.deallocate(block1).is_ok());
        assert_eq!(allocator.stack_depth(), 0);
        assert!(allocator.is_empty());
    }
    
    #[test]
    fn test_stack_out_of_order_deallocation() {
        let memory = [0u8; 4096];
        let mut allocator = StackAllocator::new(
            memory.as_ptr() as *mut u8,
            4096,
            8
        ).unwrap();
        
        let block1 = allocator.allocate(100).unwrap();
        let block2 = allocator.allocate(200).unwrap();
        
        // 尝试先释放第一个块（违反LIFO顺序）
        assert_eq!(allocator.deallocate(block1), Err(MemoryError::InvalidSize));
        
        // 正确的顺序应该成功
        assert!(allocator.deallocate(block2).is_ok());
        assert!(allocator.deallocate(block1).is_ok());
    }
    
    #[test]
    fn test_stack_marker() {
        let memory = [0u8; 4096];
        let mut allocator = StackAllocator::new(
            memory.as_ptr() as *mut u8,
            4096,
            8
        ).unwrap();
        
        // 创建标记
        let marker = allocator.create_marker();
        
        // 分配一些内存
        let _block1 = allocator.allocate(100).unwrap();
        let _block2 = allocator.allocate(200).unwrap();
        
        assert_eq!(allocator.stack_depth(), 2);
        
        // 恢复到标记
        assert!(allocator.restore_to_marker(marker).is_ok());
        assert_eq!(allocator.stack_depth(), 0);
        assert!(allocator.is_empty());
    }
    
    #[test]
    fn test_stack_scope() {
        let memory = [0u8; 4096];
        let mut allocator = StackAllocator::new(
            memory.as_ptr() as *mut u8,
            4096,
            8
        ).unwrap();
        
        {
            let mut scope = StackScope::new(&mut allocator);
            let _block1 = scope.allocate(100).unwrap();
            let _block2 = scope.allocate(200).unwrap();
            
            assert_eq!(scope.allocator.stack_depth(), 2);
        } // scope被销毁，自动恢复
        
        assert_eq!(allocator.stack_depth(), 0);
        assert!(allocator.is_empty());
    }
    
    #[test]
    fn test_stack_alignment() {
        let memory = [0u8; 4096];
        let mut allocator = StackAllocator::new(
            memory.as_ptr() as *mut u8,
            4096,
            8
        ).unwrap();
        
        // 分配对齐内存
        let block = allocator.allocate_aligned(100, 32);
        assert!(block.is_ok());
        let block = block.unwrap();
        
        // 检查对齐
        assert_eq!(block.ptr as usize % 32, 0);
        
        assert!(allocator.deallocate(block).is_ok());
    }
    
    #[test]
    fn test_stack_integrity() {
        let memory = [0u8; 4096];
        let mut allocator = StackAllocator::new(
            memory.as_ptr() as *mut u8,
            4096,
            8
        ).unwrap();
        
        // 初始完整性检查
        assert!(allocator.check_integrity().is_ok());
        
        // 分配一些内存
        let block1 = allocator.allocate(100).unwrap();
        let block2 = allocator.allocate(200).unwrap();
        
        // 检查完整性
        assert!(allocator.check_integrity().is_ok());
        
        // 释放内存
        assert!(allocator.deallocate(block2).is_ok());
        assert!(allocator.check_integrity().is_ok());
        
        assert!(allocator.deallocate(block1).is_ok());
        assert!(allocator.check_integrity().is_ok());
    }
}