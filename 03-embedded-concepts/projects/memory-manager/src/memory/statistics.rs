/// 内存统计信息
#[derive(Debug, Clone, Copy)]
pub struct MemoryStatistics {
    /// 总内存大小
    pub total_size: usize,
    /// 已分配内存大小
    pub allocated_size: usize,
    /// 空闲内存大小
    pub free_size: usize,
    /// 分配次数
    pub allocation_count: u32,
    /// 释放次数
    pub deallocation_count: u32,
    /// 碎片化比率 (0.0 - 1.0)
    pub fragmentation_ratio: f32,
}

impl MemoryStatistics {
    /// 创建新的内存统计信息
    pub fn new(total_size: usize) -> Self {
        Self {
            total_size,
            allocated_size: 0,
            free_size: total_size,
            allocation_count: 0,
            deallocation_count: 0,
            fragmentation_ratio: 0.0,
        }
    }
    
    /// 计算内存利用率 (0.0 - 1.0)
    pub fn utilization(&self) -> f32 {
        if self.total_size == 0 {
            0.0
        } else {
            self.allocated_size as f32 / self.total_size as f32
        }
    }
    
    /// 计算空闲率 (0.0 - 1.0)
    pub fn free_ratio(&self) -> f32 {
        if self.total_size == 0 {
            0.0
        } else {
            self.free_size as f32 / self.total_size as f32
        }
    }
    
    /// 计算平均分配大小
    pub fn average_allocation_size(&self) -> f32 {
        if self.allocation_count == 0 {
            0.0
        } else {
            self.allocated_size as f32 / self.allocation_count as f32
        }
    }
    
    /// 检查内存是否健康
    pub fn is_healthy(&self) -> bool {
        // 内存利用率不超过90%，碎片化率不超过30%
        self.utilization() < 0.9 && self.fragmentation_ratio < 0.3
    }
    
    /// 获取内存健康等级
    pub fn health_grade(&self) -> MemoryHealthGrade {
        let utilization = self.utilization();
        let fragmentation = self.fragmentation_ratio;
        
        if utilization < 0.5 && fragmentation < 0.1 {
            MemoryHealthGrade::Excellent
        } else if utilization < 0.7 && fragmentation < 0.2 {
            MemoryHealthGrade::Good
        } else if utilization < 0.85 && fragmentation < 0.3 {
            MemoryHealthGrade::Fair
        } else if utilization < 0.95 && fragmentation < 0.5 {
            MemoryHealthGrade::Poor
        } else {
            MemoryHealthGrade::Critical
        }
    }
}

/// 内存健康等级
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MemoryHealthGrade {
    Excellent,  // 优秀
    Good,       // 良好
    Fair,       // 一般
    Poor,       // 较差
    Critical,   // 危险
}

impl core::fmt::Display for MemoryHealthGrade {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            MemoryHealthGrade::Excellent => write!(f, "Excellent"),
            MemoryHealthGrade::Good => write!(f, "Good"),
            MemoryHealthGrade::Fair => write!(f, "Fair"),
            MemoryHealthGrade::Poor => write!(f, "Poor"),
            MemoryHealthGrade::Critical => write!(f, "Critical"),
        }
    }
}

/// 详细的内存统计信息
#[derive(Debug, Clone)]
pub struct DetailedMemoryStatistics {
    /// 基本统计信息
    pub basic: MemoryStatistics,
    /// 峰值已分配内存
    pub peak_allocated_size: usize,
    /// 最大单次分配大小
    pub max_allocation_size: usize,
    /// 最小单次分配大小
    pub min_allocation_size: usize,
    /// 分配失败次数
    pub allocation_failures: u32,
    /// 内存泄漏检测到的次数
    pub memory_leaks_detected: u32,
    /// 双重释放检测到的次数
    pub double_frees_detected: u32,
    /// 总的分配时间（微秒）
    pub total_allocation_time_us: u64,
    /// 总的释放时间（微秒）
    pub total_deallocation_time_us: u64,
}

impl DetailedMemoryStatistics {
    /// 创建新的详细统计信息
    pub fn new(total_size: usize) -> Self {
        Self {
            basic: MemoryStatistics::new(total_size),
            peak_allocated_size: 0,
            max_allocation_size: 0,
            min_allocation_size: usize::MAX,
            allocation_failures: 0,
            memory_leaks_detected: 0,
            double_frees_detected: 0,
            total_allocation_time_us: 0,
            total_deallocation_time_us: 0,
        }
    }
    
    /// 计算平均分配时间（微秒）
    pub fn average_allocation_time_us(&self) -> f64 {
        if self.basic.allocation_count == 0 {
            0.0
        } else {
            self.total_allocation_time_us as f64 / self.basic.allocation_count as f64
        }
    }
    
    /// 计算平均释放时间（微秒）
    pub fn average_deallocation_time_us(&self) -> f64 {
        if self.basic.deallocation_count == 0 {
            0.0
        } else {
            self.total_deallocation_time_us as f64 / self.basic.deallocation_count as f64
        }
    }
    
    /// 计算分配成功率
    pub fn allocation_success_rate(&self) -> f32 {
        let total_attempts = self.basic.allocation_count + self.allocation_failures;
        if total_attempts == 0 {
            1.0
        } else {
            self.basic.allocation_count as f32 / total_attempts as f32
        }
    }
    
    /// 计算内存效率分数 (0-100)
    pub fn efficiency_score(&self) -> u8 {
        let utilization_score = (self.basic.utilization() * 40.0) as u8;
        let fragmentation_score = ((1.0 - self.basic.fragmentation_ratio) * 30.0) as u8;
        let success_rate_score = (self.allocation_success_rate() * 20.0) as u8;
        let leak_penalty = core::cmp::min(self.memory_leaks_detected * 2, 10);
        let double_free_penalty = core::cmp::min(self.double_frees_detected * 3, 10);
        
        let base_score = utilization_score + fragmentation_score + success_rate_score + 10;
        let penalties = leak_penalty + double_free_penalty;
        
        if base_score > penalties {
            base_score - penalties as u8
        } else {
            0
        }
    }
}

/// 内存统计收集器
pub struct MemoryStatisticsCollector {
    detailed_stats: DetailedMemoryStatistics,
    start_time: u64,
}

impl MemoryStatisticsCollector {
    /// 创建新的统计收集器
    pub fn new(total_size: usize) -> Self {
        Self {
            detailed_stats: DetailedMemoryStatistics::new(total_size),
            start_time: 0, // 在实际实现中应该获取当前时间
        }
    }
    
    /// 记录分配事件
    pub fn record_allocation(&mut self, size: usize, time_us: u64, success: bool) {
        if success {
            self.detailed_stats.basic.allocation_count += 1;
            self.detailed_stats.basic.allocated_size += size;
            self.detailed_stats.basic.free_size -= size;
            
            if self.detailed_stats.basic.allocated_size > self.detailed_stats.peak_allocated_size {
                self.detailed_stats.peak_allocated_size = self.detailed_stats.basic.allocated_size;
            }
            
            if size > self.detailed_stats.max_allocation_size {
                self.detailed_stats.max_allocation_size = size;
            }
            
            if size < self.detailed_stats.min_allocation_size {
                self.detailed_stats.min_allocation_size = size;
            }
            
            self.detailed_stats.total_allocation_time_us += time_us;
        } else {
            self.detailed_stats.allocation_failures += 1;
        }
    }
    
    /// 记录释放事件
    pub fn record_deallocation(&mut self, size: usize, time_us: u64, success: bool) {
        if success {
            self.detailed_stats.basic.deallocation_count += 1;
            self.detailed_stats.basic.allocated_size -= size;
            self.detailed_stats.basic.free_size += size;
            self.detailed_stats.total_deallocation_time_us += time_us;
        } else {
            self.detailed_stats.double_frees_detected += 1;
        }
    }
    
    /// 记录内存泄漏
    pub fn record_memory_leak(&mut self) {
        self.detailed_stats.memory_leaks_detected += 1;
    }
    
    /// 更新碎片化比率
    pub fn update_fragmentation_ratio(&mut self, ratio: f32) {
        self.detailed_stats.basic.fragmentation_ratio = ratio;
    }
    
    /// 获取基本统计信息
    pub fn get_basic_statistics(&self) -> &MemoryStatistics {
        &self.detailed_stats.basic
    }
    
    /// 获取详细统计信息
    pub fn get_detailed_statistics(&self) -> &DetailedMemoryStatistics {
        &self.detailed_stats
    }
    
    /// 重置统计信息
    pub fn reset(&mut self) {
        let total_size = self.detailed_stats.basic.total_size;
        self.detailed_stats = DetailedMemoryStatistics::new(total_size);
        self.start_time = 0; // 重置开始时间
    }
    
    /// 生成统计报告
    pub fn generate_report(&self) -> MemoryReport {
        MemoryReport {
            basic_stats: self.detailed_stats.basic,
            detailed_stats: self.detailed_stats.clone(),
            health_grade: self.detailed_stats.basic.health_grade(),
            efficiency_score: self.detailed_stats.efficiency_score(),
            recommendations: self.generate_recommendations(),
        }
    }
    
    fn generate_recommendations(&self) -> heapless::Vec<&'static str, 8> {
        let mut recommendations = heapless::Vec::new();
        
        if self.detailed_stats.basic.utilization() > 0.9 {
            let _ = recommendations.push("Consider increasing memory pool size");
        }
        
        if self.detailed_stats.basic.fragmentation_ratio > 0.3 {
            let _ = recommendations.push("High fragmentation detected, consider memory defragmentation");
        }
        
        if self.detailed_stats.allocation_failures > 0 {
            let _ = recommendations.push("Allocation failures detected, review memory usage patterns");
        }
        
        if self.detailed_stats.memory_leaks_detected > 0 {
            let _ = recommendations.push("Memory leaks detected, review deallocation logic");
        }
        
        if self.detailed_stats.double_frees_detected > 0 {
            let _ = recommendations.push("Double free errors detected, review pointer management");
        }
        
        if self.detailed_stats.average_allocation_time_us() > 100.0 {
            let _ = recommendations.push("High allocation latency, consider optimizing allocator");
        }
        
        recommendations
    }
}

/// 内存报告
#[derive(Debug, Clone)]
pub struct MemoryReport {
    pub basic_stats: MemoryStatistics,
    pub detailed_stats: DetailedMemoryStatistics,
    pub health_grade: MemoryHealthGrade,
    pub efficiency_score: u8,
    pub recommendations: heapless::Vec<&'static str, 8>,
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_memory_statistics() {
        let mut stats = MemoryStatistics::new(1024);
        
        assert_eq!(stats.total_size, 1024);
        assert_eq!(stats.allocated_size, 0);
        assert_eq!(stats.free_size, 1024);
        assert_eq!(stats.utilization(), 0.0);
        assert_eq!(stats.free_ratio(), 1.0);
        assert!(stats.is_healthy());
        assert_eq!(stats.health_grade(), MemoryHealthGrade::Excellent);
    }
    
    #[test]
    fn test_statistics_collector() {
        let mut collector = MemoryStatisticsCollector::new(1024);
        
        // 记录成功分配
        collector.record_allocation(256, 10, true);
        let stats = collector.get_basic_statistics();
        assert_eq!(stats.allocation_count, 1);
        assert_eq!(stats.allocated_size, 256);
        assert_eq!(stats.free_size, 768);
        
        // 记录成功释放
        collector.record_deallocation(256, 5, true);
        let stats = collector.get_basic_statistics();
        assert_eq!(stats.deallocation_count, 1);
        assert_eq!(stats.allocated_size, 0);
        assert_eq!(stats.free_size, 1024);
        
        // 记录分配失败
        collector.record_allocation(2048, 15, false);
        let detailed = collector.get_detailed_statistics();
        assert_eq!(detailed.allocation_failures, 1);
    }
    
    #[test]
    fn test_memory_report() {
        let mut collector = MemoryStatisticsCollector::new(1024);
        collector.record_allocation(512, 20, true);
        collector.update_fragmentation_ratio(0.1);
        
        let report = collector.generate_report();
        assert_eq!(report.basic_stats.allocated_size, 512);
        assert_eq!(report.health_grade, MemoryHealthGrade::Good);
        assert!(report.efficiency_score > 50);
    }
}