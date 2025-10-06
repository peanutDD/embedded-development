//! 性能分析模块

use crate::{Scheduler, SchedulerResult, SchedulerStatistics};
use core::sync::atomic::{AtomicU32, Ordering};
use core::cell::RefCell;
use alloc::vec::Vec;
use alloc::format;

/// 实时性能分析器
pub struct PerformanceAnalyzer {
    /// 调度器引用
    scheduler: &'static dyn Scheduler,
    /// 采样周期（微秒）
    sampling_period: u32,
    /// 历史统计数据
    history: RefCell<Vec<SchedulerStatistics>>,
    /// 最大历史记录数
    max_history: usize,
    /// 分析开始时间
    start_time: AtomicU32,
    /// 是否正在分析
    running: AtomicU32,
}

impl PerformanceAnalyzer {
    /// 创建新的性能分析器
    pub fn new(scheduler: &'static dyn Scheduler, sampling_period: u32, max_history: usize) -> Self {
        Self {
            scheduler,
            sampling_period,
            history: RefCell::new(Vec::with_capacity(max_history)),
            max_history,
            start_time: AtomicU32::new(0),
            running: AtomicU32::new(0),
        }
    }

    /// 开始性能分析
    pub fn start(&self, current_time: u32) {
        self.start_time.store(current_time, Ordering::Relaxed);
        self.running.store(1, Ordering::Relaxed);
        self.history.borrow_mut().clear();
    }

    /// 停止性能分析
    pub fn stop(&self) {
        self.running.store(0, Ordering::Relaxed);
    }

    /// 更新性能分析数据
    pub fn update(&self, current_time: u32) -> bool {
        if self.running.load(Ordering::Relaxed) == 0 {
            return false;
        }
        
        // 检查是否到达采样周期
        let start_time = self.start_time.load(Ordering::Relaxed);
        let elapsed = current_time.wrapping_sub(start_time);
        
        if elapsed >= self.sampling_period {
            // 保存当前统计信息
            let statistics = self.scheduler.statistics();
            let mut history = self.history.borrow_mut();
            
            if history.len() >= self.max_history {
                history.remove(0);
            }
            
            history.push(statistics);
            
            // 更新开始时间
            self.start_time.store(current_time, Ordering::Relaxed);
            
            return true;
        }
        
        false
    }

    /// 获取历史统计数据
    pub fn get_history(&self) -> Vec<SchedulerStatistics> {
        self.history.borrow().clone()
    }

    /// 计算平均CPU利用率
    pub fn calculate_average_cpu_utilization(&self) -> f64 {
        let history = self.history.borrow();
        
        if history.is_empty() {
            return 0.0;
        }
        
        let sum: f64 = history.iter().map(|s| s.cpu_utilization as f64).sum();
        sum / history.len() as f64
    }

    /// 计算平均上下文切换次数
    pub fn calculate_average_context_switches(&self) -> u32 {
        let history = self.history.borrow();
        
        if history.is_empty() {
            return 0;
        }
        
        let sum: u32 = history.iter().map(|s| s.context_switches).sum();
        sum / history.len() as u32
    }

    /// 计算平均截止期错过次数
    pub fn calculate_average_deadline_misses(&self) -> u32 {
        let history = self.history.borrow();
        
        if history.is_empty() {
            return 0;
        }
        
        let sum: u32 = history.iter().map(|s| s.deadline_misses).sum();
        sum / history.len() as u32
    }

    /// 获取分析持续时间
    pub fn get_duration(&self, current_time: u32) -> u32 {
        if self.running.load(Ordering::Relaxed) == 0 {
            return 0;
        }
        
        current_time.wrapping_sub(self.start_time.load(Ordering::Relaxed))
    }

    /// 获取最大响应时间
    pub fn get_max_response_time(&self) -> u32 {
        let history = self.history.borrow();
        
        if history.is_empty() {
            return 0;
        }
        
        history.iter().map(|s| s.max_response_time_us).max().unwrap_or(0)
    }

    /// 生成性能报告
    pub fn generate_report(&self, current_time: u32) -> PerformanceReport {
        PerformanceReport {
            duration: self.get_duration(current_time),
            average_cpu_utilization: self.calculate_average_cpu_utilization(),
            average_context_switches: self.calculate_average_context_switches(),
            average_deadline_misses: self.calculate_average_deadline_misses(),
            max_response_time_us: self.get_max_response_time(),
            sample_count: self.history.borrow().len() as u32,
        }
    }

    /// 重置分析器
    pub fn reset(&self) {
        self.stop();
        self.history.borrow_mut().clear();
        self.start_time.store(0, Ordering::Relaxed);
    }
}

/// 性能报告结构体
pub struct PerformanceReport {
    /// 分析持续时间（微秒）
    pub duration: u32,
    /// 平均CPU利用率
    pub average_cpu_utilization: f64,
    /// 平均上下文切换次数
    pub average_context_switches: u32,
    /// 平均截止期错过次数
    pub average_deadline_misses: u32,
    /// 最大响应时间（微秒）
    pub max_response_time_us: u32,
    /// 采样数量
    pub sample_count: u32,
}

impl PerformanceReport {
    /// 格式化性能报告为字符串
    pub fn format(&self) -> alloc::string::String {
        let mut report = alloc::string::String::new();
        
        report.push_str("===== 性能分析报告 =====\n");
        report.push_str(&format!("持续时间: {} 微秒\n", self.duration));
        report.push_str(&format!("平均CPU利用率: {:.2}%\n", self.average_cpu_utilization * 100.0));
        report.push_str(&format!("平均上下文切换次数: {}\n", self.average_context_switches));
        report.push_str(&format!("平均截止期错过次数: {}\n", self.average_deadline_misses));
        report.push_str(&format!("最大响应时间: {} 微秒\n", self.max_response_time_us));
        report.push_str(&format!("采样数量: {}\n", self.sample_count));
        report.push_str("======================");
        
        report
    }
}

/// 可调度性分析器
pub struct SchedulabilityAnalyzer {
    /// 保存的任务配置数据
    task_data: RefCell<Vec<TaskAnalysisData>>,
}

/// 任务分析数据
#[derive(Debug, Clone, Copy)]
pub struct TaskAnalysisData {
    /// 任务周期（微秒）
    pub period: u32,
    /// 任务执行时间（微秒）
    pub execution_time: u32,
    /// 任务截止期（微秒）
    pub deadline: u32,
    /// 任务优先级
    pub priority: u8,
}

impl SchedulabilityAnalyzer {
    /// 创建新的可调度性分析器
    pub const fn new() -> Self {
        Self {
            task_data: RefCell::new(Vec::new()),
        }
    }

    /// 添加任务数据
    pub fn add_task(&self, data: TaskAnalysisData) {
        self.task_data.borrow_mut().push(data);
    }

    /// 清除所有任务数据
    pub fn clear(&self) {
        self.task_data.borrow_mut().clear();
    }

    /// 使用RM算法检查可调度性
    pub fn check_rm_schedulability(&self) -> bool {
        let tasks = self.task_data.borrow();
        
        if tasks.is_empty() {
            return true;
        }
        
        // 按照周期排序（RM算法按周期排序，周期越小优先级越高）
        let mut sorted_tasks = tasks.clone();
        sorted_tasks.sort_by(|a, b| a.period.cmp(&b.period));
        
        // 计算总利用率
        let sum_utilization: f64 = sorted_tasks
            .iter()
            .map(|t| t.execution_time as f64 / t.period as f64)
            .sum();
        
        let n = sorted_tasks.len() as f64;
        // 在no_std环境中，我们实现一个简单的幂函数近似
        let exponent = 1.0 / n;
        // 使用泰勒展开近似计算2^exponent
        let ln2 = 0.69314718056; // ln(2)的近似值
        let x = exponent * ln2;
        let pow_approx = 1.0 + x + x*x/2.0 + x*x*x/6.0 + x*x*x*x/24.0; // e^x的泰勒展开前5项
        let rm_bound = n * (pow_approx - 1.0);
        
        // 首先检查Liu and Layland边界
        if sum_utilization > rm_bound {
            // 如果超过边界，执行精确的可调度性测试
            return self.check_exact_schedulability(&sorted_tasks);
        }
        
        true
    }

    /// 使用EDF算法检查可调度性
    pub fn check_edf_schedulability(&self) -> bool {
        let tasks = self.task_data.borrow();
        
        if tasks.is_empty() {
            return true;
        }
        
        // EDF的可调度性条件：总利用率 <= 1.0
        let sum_utilization: f64 = tasks
            .iter()
            .map(|t| t.execution_time as f64 / t.deadline.max(t.period) as f64)
            .sum();
        
        sum_utilization <= 1.0
    }

    /// 精确的可调度性测试
    fn check_exact_schedulability(&self, sorted_tasks: &[TaskAnalysisData]) -> bool {
        // 实现Response Time Analysis (RTA)算法
        for i in 0..sorted_tasks.len() {
            let mut response_time = sorted_tasks[i].execution_time;
            let deadline = sorted_tasks[i].deadline.max(sorted_tasks[i].period);
            
            loop {
                let mut demand = sorted_tasks[i].execution_time;
                
                // 计算所有更高优先级任务的干扰
                for j in 0..i {
                    demand += (response_time + sorted_tasks[j].period - 1) / sorted_tasks[j].period * sorted_tasks[j].execution_time;
                }
                
                if demand == response_time || demand > deadline {
                    break;
                }
                
                response_time = demand;
            }
            
            if response_time > deadline {
                return false;
            }
        }
        
        true
    }

    /// 获取总CPU利用率
    pub fn get_total_cpu_utilization(&self) -> f64 {
        let tasks = self.task_data.borrow();
        
        if tasks.is_empty() {
            return 0.0;
        }
        
        tasks
            .iter()
            .map(|t| t.execution_time as f64 / t.period as f64)
            .sum()
    }
}