use heapless::Vec;

/// 位图数据结构
/// 
/// 用于高效地跟踪大量布尔值状态，常用于内存分配器中跟踪块的分配状态
pub struct Bitmap {
    bits: Vec<u32, 64>, // 使用u32数组存储位
    size: usize,        // 总位数
}

impl Bitmap {
    /// 创建新的位图
    /// 
    /// # 参数
    /// - `size`: 位图大小（位数）
    pub fn new(size: usize) -> Self {
        let words_needed = (size + 31) / 32; // 向上取整
        let mut bits = Vec::new();
        
        // 初始化所有位为0
        for _ in 0..words_needed {
            let _ = bits.push(0);
        }
        
        Self { bits, size }
    }
    
    /// 设置指定位的值
    /// 
    /// # 参数
    /// - `index`: 位索引
    /// - `value`: 要设置的值
    pub fn set(&mut self, index: usize, value: bool) {
        if index >= self.size {
            return;
        }
        
        let word_index = index / 32;
        let bit_index = index % 32;
        
        if let Some(word) = self.bits.get_mut(word_index) {
            if value {
                *word |= 1 << bit_index;
            } else {
                *word &= !(1 << bit_index);
            }
        }
    }
    
    /// 获取指定位的值
    /// 
    /// # 参数
    /// - `index`: 位索引
    /// 
    /// # 返回
    /// 位的值，如果索引超出范围返回false
    pub fn get(&self, index: usize) -> bool {
        if index >= self.size {
            return false;
        }
        
        let word_index = index / 32;
        let bit_index = index % 32;
        
        if let Some(word) = self.bits.get(word_index) {
            (word & (1 << bit_index)) != 0
        } else {
            false
        }
    }
    
    /// 翻转指定位的值
    /// 
    /// # 参数
    /// - `index`: 位索引
    pub fn flip(&mut self, index: usize) {
        if index >= self.size {
            return;
        }
        
        let word_index = index / 32;
        let bit_index = index % 32;
        
        if let Some(word) = self.bits.get_mut(word_index) {
            *word ^= 1 << bit_index;
        }
    }
    
    /// 清除所有位（设置为0）
    pub fn clear_all(&mut self) {
        for word in &mut self.bits {
            *word = 0;
        }
    }
    
    /// 设置所有位（设置为1）
    pub fn set_all(&mut self) {
        for (i, word) in self.bits.iter_mut().enumerate() {
            if (i + 1) * 32 <= self.size {
                *word = u32::MAX;
            } else {
                // 最后一个字可能不完整
                let remaining_bits = self.size - i * 32;
                *word = (1u32 << remaining_bits) - 1;
            }
        }
    }
    
    /// 查找第一个设置为指定值的位
    /// 
    /// # 参数
    /// - `value`: 要查找的值
    /// 
    /// # 返回
    /// 第一个匹配位的索引，如果没找到返回None
    pub fn find_first(&self, value: bool) -> Option<usize> {
        for (word_index, &word) in self.bits.iter().enumerate() {
            let target_word = if value { word } else { !word };
            
            if target_word != 0 {
                let bit_index = target_word.trailing_zeros() as usize;
                let global_index = word_index * 32 + bit_index;
                
                if global_index < self.size {
                    return Some(global_index);
                }
            }
        }
        
        None
    }
    
    /// 查找第一个连续的n个指定值的位
    /// 
    /// # 参数
    /// - `value`: 要查找的值
    /// - `count`: 连续位的数量
    /// 
    /// # 返回
    /// 第一个连续区域的起始索引，如果没找到返回None
    pub fn find_consecutive(&self, value: bool, count: usize) -> Option<usize> {
        if count == 0 {
            return Some(0);
        }
        
        let mut consecutive_count = 0;
        let mut start_index = 0;
        
        for i in 0..self.size {
            if self.get(i) == value {
                if consecutive_count == 0 {
                    start_index = i;
                }
                consecutive_count += 1;
                
                if consecutive_count >= count {
                    return Some(start_index);
                }
            } else {
                consecutive_count = 0;
            }
        }
        
        None
    }
    
    /// 设置连续的多个位
    /// 
    /// # 参数
    /// - `start`: 起始索引
    /// - `count`: 位数
    /// - `value`: 要设置的值
    pub fn set_range(&mut self, start: usize, count: usize, value: bool) {
        for i in start..core::cmp::min(start + count, self.size) {
            self.set(i, value);
        }
    }
    
    /// 计算设置为true的位数
    pub fn count_ones(&self) -> usize {
        let mut count = 0;
        
        for (i, &word) in self.bits.iter().enumerate() {
            if (i + 1) * 32 <= self.size {
                count += word.count_ones() as usize;
            } else {
                // 最后一个字可能不完整
                let remaining_bits = self.size - i * 32;
                let mask = (1u32 << remaining_bits) - 1;
                count += (word & mask).count_ones() as usize;
            }
        }
        
        count
    }
    
    /// 计算设置为false的位数
    pub fn count_zeros(&self) -> usize {
        self.size - self.count_ones()
    }
    
    /// 获取位图大小
    pub fn size(&self) -> usize {
        self.size
    }
    
    /// 检查位图是否为空（所有位都是false）
    pub fn is_empty(&self) -> bool {
        self.count_ones() == 0
    }
    
    /// 检查位图是否已满（所有位都是true）
    pub fn is_full(&self) -> bool {
        self.count_ones() == self.size
    }
    
    /// 获取位图的利用率（0.0到1.0）
    pub fn utilization(&self) -> f32 {
        if self.size == 0 {
            0.0
        } else {
            self.count_ones() as f32 / self.size as f32
        }
    }
}

impl core::fmt::Debug for Bitmap {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "Bitmap {{ size: {}, ones: {}, zeros: {} }}", 
               self.size, self.count_ones(), self.count_zeros())
    }
}

/// 位图迭代器
pub struct BitmapIterator<'a> {
    bitmap: &'a Bitmap,
    current_index: usize,
}

impl<'a> Iterator for BitmapIterator<'a> {
    type Item = bool;
    
    fn next(&mut self) -> Option<Self::Item> {
        if self.current_index < self.bitmap.size() {
            let value = self.bitmap.get(self.current_index);
            self.current_index += 1;
            Some(value)
        } else {
            None
        }
    }
}

impl Bitmap {
    /// 创建迭代器
    pub fn iter(&self) -> BitmapIterator {
        BitmapIterator {
            bitmap: self,
            current_index: 0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_bitmap_creation() {
        let bitmap = Bitmap::new(100);
        assert_eq!(bitmap.size(), 100);
        assert!(bitmap.is_empty());
        assert!(!bitmap.is_full());
    }
    
    #[test]
    fn test_set_and_get() {
        let mut bitmap = Bitmap::new(64);
        
        assert!(!bitmap.get(10));
        bitmap.set(10, true);
        assert!(bitmap.get(10));
        
        bitmap.set(10, false);
        assert!(!bitmap.get(10));
    }
    
    #[test]
    fn test_flip() {
        let mut bitmap = Bitmap::new(32);
        
        assert!(!bitmap.get(5));
        bitmap.flip(5);
        assert!(bitmap.get(5));
        bitmap.flip(5);
        assert!(!bitmap.get(5));
    }
    
    #[test]
    fn test_clear_and_set_all() {
        let mut bitmap = Bitmap::new(50);
        
        bitmap.set_all();
        assert_eq!(bitmap.count_ones(), 50);
        assert!(bitmap.is_full());
        
        bitmap.clear_all();
        assert_eq!(bitmap.count_ones(), 0);
        assert!(bitmap.is_empty());
    }
    
    #[test]
    fn test_find_first() {
        let mut bitmap = Bitmap::new(64);
        
        bitmap.set(10, true);
        bitmap.set(20, true);
        
        assert_eq!(bitmap.find_first(true), Some(10));
        assert_eq!(bitmap.find_first(false), Some(0));
    }
    
    #[test]
    fn test_find_consecutive() {
        let mut bitmap = Bitmap::new(64);
        
        // 设置位15-19为true
        for i in 15..20 {
            bitmap.set(i, true);
        }
        
        assert_eq!(bitmap.find_consecutive(true, 3), Some(15));
        assert_eq!(bitmap.find_consecutive(true, 5), Some(15));
        assert_eq!(bitmap.find_consecutive(true, 6), None);
        assert_eq!(bitmap.find_consecutive(false, 10), Some(0));
    }
    
    #[test]
    fn test_set_range() {
        let mut bitmap = Bitmap::new(64);
        
        bitmap.set_range(10, 5, true);
        
        for i in 10..15 {
            assert!(bitmap.get(i));
        }
        assert!(!bitmap.get(9));
        assert!(!bitmap.get(15));
    }
    
    #[test]
    fn test_count() {
        let mut bitmap = Bitmap::new(100);
        
        bitmap.set_range(10, 20, true);
        
        assert_eq!(bitmap.count_ones(), 20);
        assert_eq!(bitmap.count_zeros(), 80);
        assert_eq!(bitmap.utilization(), 0.2);
    }
    
    #[test]
    fn test_iterator() {
        let mut bitmap = Bitmap::new(10);
        bitmap.set(2, true);
        bitmap.set(5, true);
        bitmap.set(8, true);
        
        let values: Vec<bool> = bitmap.iter().collect();
        assert_eq!(values.len(), 10);
        assert!(!values[0]);
        assert!(!values[1]);
        assert!(values[2]);
        assert!(!values[3]);
        assert!(!values[4]);
        assert!(values[5]);
        assert!(!values[6]);
        assert!(!values[7]);
        assert!(values[8]);
        assert!(!values[9]);
    }
}