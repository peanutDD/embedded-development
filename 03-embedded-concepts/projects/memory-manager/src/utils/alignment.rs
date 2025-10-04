/// 向上对齐到指定边界
/// 
/// # 参数
/// - `value`: 要对齐的值
/// - `alignment`: 对齐边界（必须是2的幂次）
/// 
/// # 返回
/// 对齐后的值
pub fn align_up(value: usize, alignment: usize) -> usize {
    debug_assert!(alignment.is_power_of_two(), "Alignment must be a power of 2");
    (value + alignment - 1) & !(alignment - 1)
}

/// 向下对齐到指定边界
/// 
/// # 参数
/// - `value`: 要对齐的值
/// - `alignment`: 对齐边界（必须是2的幂次）
/// 
/// # 返回
/// 对齐后的值
pub fn align_down(value: usize, alignment: usize) -> usize {
    debug_assert!(alignment.is_power_of_two(), "Alignment must be a power of 2");
    value & !(alignment - 1)
}

/// 检查值是否已对齐
/// 
/// # 参数
/// - `value`: 要检查的值
/// - `alignment`: 对齐边界（必须是2的幂次）
/// 
/// # 返回
/// 如果已对齐返回true，否则返回false
pub fn is_aligned(value: usize, alignment: usize) -> bool {
    debug_assert!(alignment.is_power_of_two(), "Alignment must be a power of 2");
    (value & (alignment - 1)) == 0
}

/// 计算对齐偏移量
/// 
/// # 参数
/// - `value`: 当前值
/// - `alignment`: 对齐边界（必须是2的幂次）
/// 
/// # 返回
/// 需要添加的偏移量以达到对齐
pub fn alignment_offset(value: usize, alignment: usize) -> usize {
    debug_assert!(alignment.is_power_of_two(), "Alignment must be a power of 2");
    let aligned = align_up(value, alignment);
    aligned - value
}

/// 内存对齐辅助结构
pub struct AlignmentHelper {
    alignment: usize,
}

impl AlignmentHelper {
    /// 创建新的对齐辅助器
    pub fn new(alignment: usize) -> Self {
        debug_assert!(alignment.is_power_of_two(), "Alignment must be a power of 2");
        Self { alignment }
    }
    
    /// 向上对齐
    pub fn align_up(&self, value: usize) -> usize {
        align_up(value, self.alignment)
    }
    
    /// 向下对齐
    pub fn align_down(&self, value: usize) -> usize {
        align_down(value, self.alignment)
    }
    
    /// 检查是否对齐
    pub fn is_aligned(&self, value: usize) -> bool {
        is_aligned(value, self.alignment)
    }
    
    /// 计算对齐偏移
    pub fn offset(&self, value: usize) -> usize {
        alignment_offset(value, self.alignment)
    }
    
    /// 获取对齐值
    pub fn alignment(&self) -> usize {
        self.alignment
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_align_up() {
        assert_eq!(align_up(0, 8), 0);
        assert_eq!(align_up(1, 8), 8);
        assert_eq!(align_up(7, 8), 8);
        assert_eq!(align_up(8, 8), 8);
        assert_eq!(align_up(9, 8), 16);
        assert_eq!(align_up(15, 8), 16);
        assert_eq!(align_up(16, 8), 16);
    }
    
    #[test]
    fn test_align_down() {
        assert_eq!(align_down(0, 8), 0);
        assert_eq!(align_down(1, 8), 0);
        assert_eq!(align_down(7, 8), 0);
        assert_eq!(align_down(8, 8), 8);
        assert_eq!(align_down(9, 8), 8);
        assert_eq!(align_down(15, 8), 8);
        assert_eq!(align_down(16, 8), 16);
    }
    
    #[test]
    fn test_is_aligned() {
        assert!(is_aligned(0, 8));
        assert!(!is_aligned(1, 8));
        assert!(!is_aligned(7, 8));
        assert!(is_aligned(8, 8));
        assert!(!is_aligned(9, 8));
        assert!(!is_aligned(15, 8));
        assert!(is_aligned(16, 8));
    }
    
    #[test]
    fn test_alignment_offset() {
        assert_eq!(alignment_offset(0, 8), 0);
        assert_eq!(alignment_offset(1, 8), 7);
        assert_eq!(alignment_offset(7, 8), 1);
        assert_eq!(alignment_offset(8, 8), 0);
        assert_eq!(alignment_offset(9, 8), 7);
    }
    
    #[test]
    fn test_alignment_helper() {
        let helper = AlignmentHelper::new(16);
        
        assert_eq!(helper.align_up(10), 16);
        assert_eq!(helper.align_down(30), 16);
        assert!(helper.is_aligned(32));
        assert!(!helper.is_aligned(33));
        assert_eq!(helper.offset(10), 6);
        assert_eq!(helper.alignment(), 16);
    }
}