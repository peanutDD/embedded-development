//! 内存跟踪器实现

use crate::{MemoryError, MemoryResult};
use core::ptr::NonNull;

/// 内存分配记录
struct AllocationRecord {
    /// 分配的内存指针
    ptr: *mut u8,
    /// 分配的内存大小
    size: usize,
    /// 内存对齐要求
    alignment: usize,
    /// 是否已释放
    freed: bool,
}

/// 内存跟踪器
/// 
/// 用于检测内存泄漏、重复释放和内存损坏
pub struct MemoryTracker {
    /// 分配记录数组
    records: alloc::vec::Vec<AllocationRecord>,
    /// 最大记录数量
    max_records: usize,
    /// 下一个可用的记录索引
    next_index: usize,
}

impl MemoryTracker {
    /// 创建新的内存跟踪器
    pub fn new(max_records: usize) -> Self {
        Self {
            records: alloc::vec::Vec::with_capacity(max_records),
            max_records,
            next_index: 0,
        }
    }

    /// 跟踪内存分配
    pub fn track_allocation(&mut self, ptr: *mut u8, size: usize, alignment: usize) -> MemoryResult<()> {
        // 检查是否超过最大记录数量
        if self.records.len() >= self.max_records {
            return Err(MemoryError::OutOfMemory);
        }

        // 检查指针是否有效
        if ptr.is_null() {
            return Err(MemoryError::NullPointer);
        }

        // 检查指针是否已经被跟踪
        for record in &self.records {
            if record.ptr == ptr && !record.freed {
                return Err(MemoryError::Corruption);
            }
        }

        // 添加新的分配记录
        self.records.push(AllocationRecord {
            ptr,
            size,
            alignment,
            freed: false,
        });

        Ok(())
    }

    /// 检查内存释放
    pub fn check_deallocation(&mut self, ptr: *mut u8, size: usize) -> MemoryResult<()> {
        // 检查指针是否有效
        if ptr.is_null() {
            return Err(MemoryError::NullPointer);
        }

        // 查找对应的分配记录
        let mut found = false;
        for record in &mut self.records {
            if record.ptr == ptr {
                found = true;
                
                // 检查是否已经释放
                if record.freed {
                    return Err(MemoryError::DoubleFree);
                }
                
                // 检查释放的大小是否匹配
                if record.size < size {
                    return Err(MemoryError::Corruption);
                }
                
                // 标记为已释放
                record.freed = true;
                break;
            }
        }

        // 如果没有找到对应的记录，说明释放了未分配的内存
        if !found {
            return Err(MemoryError::Corruption);
        }

        Ok(())
    }

    /// 检查内存完整性
    pub fn check_integrity(&self) -> MemoryResult<()> {
        // 检查记录是否有效
        for (i, record) in self.records.iter().enumerate() {
            if record.ptr.is_null() {
                return Err(MemoryError::Corruption);
            }
            
            if record.size == 0 {
                return Err(MemoryError::Corruption);
            }
            
            if record.alignment == 0 || !record.alignment.is_power_of_two() {
                return Err(MemoryError::Corruption);
            }
        }

        // 检查记录之间是否重叠
        for i in 0..self.records.len() {
            let record_i = &self.records[i];
            if record_i.freed {
                continue;
            }
            
            let start_i = record_i.ptr as usize;
            let end_i = start_i + record_i.size;
            
            for j in (i + 1)..self.records.len() {
                let record_j = &self.records[j];
                if record_j.freed {
                    continue;
                }
                
                let start_j = record_j.ptr as usize;
                let end_j = start_j + record_j.size;
                
                // 检查是否重叠
                if !(end_i <= start_j || end_j <= start_i) {
                    return Err(MemoryError::Corruption);
                }
            }
        }

        Ok(())
    }

    /// 获取内存泄漏报告
    pub fn get_leaks(&self) -> alloc::vec::Vec<(usize, *mut u8, usize)> {
        let mut leaks = alloc::vec::Vec::new();
        
        for (i, record) in self.records.iter().enumerate() {
            if !record.freed {
                leaks.push((i, record.ptr, record.size));
            }
        }
        
        leaks
    }

    /// 检查是否有内存泄漏
    pub fn has_leaks(&self) -> bool {
        for record in &self.records {
            if !record.freed {
                return true;
            }
        }
        false
    }

    /// 获取分配记录数量
    pub fn record_count(&self) -> usize {
        self.records.len()
    }

    /// 获取活动分配数量
    pub fn active_allocations(&self) -> usize {
        self.records.iter().filter(|r| !r.freed).count()
    }

    /// 重置跟踪器
    pub fn reset(&mut self) {
        self.records.clear();
        self.next_index = 0;
    }
}