//! 算法模块

use crate::*;
use heapless::Vec;

/// 滤波算法
pub mod filters {
    use super::*;
    
    /// 低通滤波器
    pub struct LowPassFilter {
        alpha: f32,
        previous_output: f32,
        initialized: bool,
    }
    
    impl LowPassFilter {
        pub fn new(alpha: f32) -> Self {
            Self {
                alpha: alpha.clamp(0.0, 1.0),
                previous_output: 0.0,
                initialized: false,
            }
        }
        
        pub fn update(&mut self, input: f32) -> f32 {
            if !self.initialized {
                self.previous_output = input;
                self.initialized = true;
                return input;
            }
            
            self.previous_output = self.alpha * input + (1.0 - self.alpha) * self.previous_output;
            self.previous_output
        }
        
        pub fn reset(&mut self) {
            self.previous_output = 0.0;
            self.initialized = false;
        }
    }
    
    /// 卡尔曼滤波器（简化版）
    pub struct KalmanFilter {
        q: f32, // 过程噪声协方差
        r: f32, // 测量噪声协方差
        x: f32, // 状态估计
        p: f32, // 估计误差协方差
        k: f32, // 卡尔曼增益
    }
    
    impl KalmanFilter {
        pub fn new(q: f32, r: f32, initial_value: f32) -> Self {
            Self {
                q,
                r,
                x: initial_value,
                p: 1.0,
                k: 0.0,
            }
        }
        
        pub fn update(&mut self, measurement: f32) -> f32 {
            // 预测步骤
            self.p += self.q;
            
            // 更新步骤
            self.k = self.p / (self.p + self.r);
            self.x += self.k * (measurement - self.x);
            self.p *= 1.0 - self.k;
            
            self.x
        }
    }
}

/// 控制算法
pub mod control {
    use super::*;
    
    /// PID控制器
    pub struct PidController {
        kp: f32,
        ki: f32,
        kd: f32,
        integral: f32,
        previous_error: f32,
        output_min: f32,
        output_max: f32,
    }
    
    impl PidController {
        pub fn new(kp: f32, ki: f32, kd: f32) -> Self {
            Self {
                kp,
                ki,
                kd,
                integral: 0.0,
                previous_error: 0.0,
                output_min: f32::NEG_INFINITY,
                output_max: f32::INFINITY,
            }
        }
        
        pub fn set_output_limits(&mut self, min: f32, max: f32) {
            self.output_min = min;
            self.output_max = max;
        }
        
        pub fn update(&mut self, setpoint: f32, measurement: f32, dt: f32) -> f32 {
            let error = setpoint - measurement;
            
            // 积分项
            self.integral += error * dt;
            
            // 微分项
            let derivative = (error - self.previous_error) / dt;
            
            // PID输出
            let output = self.kp * error + self.ki * self.integral + self.kd * derivative;
            
            // 限制输出范围
            let clamped_output = output.clamp(self.output_min, self.output_max);
            
            // 积分饱和处理
            if output != clamped_output {
                self.integral -= error * dt;
            }
            
            self.previous_error = error;
            clamped_output
        }
        
        pub fn reset(&mut self) {
            self.integral = 0.0;
            self.previous_error = 0.0;
        }
    }
}

/// 数据处理算法
pub mod data_processing {
    use super::*;
    
    /// 统计信息
    #[derive(Debug, Clone)]
    pub struct Statistics {
        pub mean: f32,
        pub variance: f32,
        pub std_dev: f32,
        pub min: f32,
        pub max: f32,
        pub count: usize,
    }
    
    /// 计算统计信息
    pub fn calculate_statistics(data: &[f32]) -> Option<Statistics> {
        if data.is_empty() {
            return None;
        }
        
        let count = data.len();
        let sum: f32 = data.iter().sum();
        let mean = sum / count as f32;
        
        let variance = data.iter()
            .map(|x| (x - mean) * (x - mean))
            .sum::<f32>() / count as f32;
        
        // 在no_std环境中，我们需要使用libm或者简化实现
        let std_dev = if variance > 0.0 {
            // 简化的平方根近似
            let mut x = variance;
            for _ in 0..10 {
                x = (x + variance / x) / 2.0;
            }
            x
        } else {
            0.0
        };
        let min = data.iter().fold(f32::INFINITY, |a, &b| a.min(b));
        let max = data.iter().fold(f32::NEG_INFINITY, |a, &b| a.max(b));
        
        Some(Statistics {
            mean,
            variance,
            std_dev,
            min,
            max,
            count,
        })
    }
    
    /// 异常值检测
    pub fn detect_outliers(data: &[f32], threshold: f32) -> Vec<usize, 32> {
        let mut outliers = Vec::new();
        
        if let Some(stats) = calculate_statistics(data) {
            for (i, &value) in data.iter().enumerate() {
                let z_score = (value - stats.mean).abs() / stats.std_dev;
                if z_score > threshold {
                    let _ = outliers.push(i);
                }
            }
        }
        
        outliers
    }
}