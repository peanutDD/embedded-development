# Industrial HMI (Human Machine Interface)

基于STM32F4的工业人机界面系统，提供触摸屏操作、数据可视化、报警管理和趋势分析功能。

## 项目特性

### 核心功能
- **触摸屏界面**: 支持ILI9341 TFT显示屏和XPT2046触摸控制器
- **多页面导航**: 主页、报警、趋势、设置、诊断页面
- **实时数据显示**: 温度、压力、流量、液位等工业参数
- **报警管理**: 多级报警系统，支持确认和历史记录
- **趋势分析**: 实时数据趋势图表显示
- **系统诊断**: 设备状态监控和故障诊断

### 技术特性
- **高性能图形**: 基于embedded-graphics的2D图形渲染
- **触摸交互**: 支持按钮点击和区域选择
- **数据管理**: 实时数据采集、存储和处理
- **模块化设计**: 可扩展的页面和功能模块
- **低功耗**: 优化的显示更新和系统管理

## 硬件连接

### 显示屏连接 (ILI9341)
```
STM32F4    ILI9341
PA5    ->  SCK
PA6    ->  MISO
PA7    ->  MOSI
PB0    ->  CS
PB1    ->  DC
PB2    ->  RST
3.3V   ->  VCC
GND    ->  GND
```

### 触摸屏连接 (XPT2046)
```
STM32F4    XPT2046
PA5    ->  DCLK
PA6    ->  DOUT
PA7    ->  DIN
PC0    ->  CS
PC1    ->  IRQ
3.3V   ->  VCC
GND    ->  GND
```

## 软件架构

### 主要模块

#### 1. HMI系统核心
```rust
pub struct IndustrialHmi<SPI, CS, DC, RST, TOUCH_CS, TOUCH_IRQ> {
    display: Ili9341<SPI, CS, DC, RST>,
    touch: Xpt2046<SPI, TOUCH_CS>,
    touch_irq: TOUCH_IRQ,
    config: HmiConfig,
    current_page: ScreenPage,
    data_points: Vec<DataPoint, 32>,
    alarms: Vec<AlarmInfo, 16>,
    trends: Vec<TrendData, 8>,
    // ...
}
```

#### 2. 页面管理
- **主页面**: 实时数据显示和导航
- **报警页面**: 报警列表和确认操作
- **趋势页面**: 历史数据趋势图
- **设置页面**: 系统参数配置
- **诊断页面**: 系统状态和故障信息

#### 3. 数据结构
```rust
// 数据点
pub struct DataPoint {
    pub tag: String<32>,
    pub value: f32,
    pub unit: String<8>,
    pub timestamp: u32,
    pub quality: bool,
}

// 报警信息
pub struct AlarmInfo {
    pub id: u16,
    pub level: AlarmLevel,
    pub message: String<64>,
    pub timestamp: u32,
    pub acknowledged: bool,
}

// 趋势数据
pub struct TrendData {
    pub tag: String<32>,
    pub values: Vec<f32, 100>,
    pub timestamps: Vec<u32, 100>,
    pub min_value: f32,
    pub max_value: f32,
}
```

## 构建和烧录

### 环境准备
```bash
# 安装Rust工具链
rustup target add thumbv7em-none-eabihf

# 安装probe-rs
cargo install probe-rs --features cli

# 安装cargo-embed
cargo install cargo-embed
```

### 编译项目
```bash
# 编译主程序
cargo build --release

# 编译特定测试程序
cargo build --bin display_test --release
cargo build --bin touch_test --release
```

### 烧录程序
```bash
# 烧录主程序
cargo embed --release

# 或使用probe-rs
probe-rs run --chip STM32F407VGTx target/thumbv7em-none-eabihf/release/industrial_hmi
```

## 系统配置

### HMI配置
```rust
pub struct HmiConfig {
    pub screen_width: u16,          // 屏幕宽度: 240
    pub screen_height: u16,         // 屏幕高度: 320
    pub touch_threshold: u16,       // 触摸阈值: 100
    pub update_interval_ms: u32,    // 更新间隔: 100ms
    pub alarm_timeout_ms: u32,      // 报警超时: 5000ms
    pub data_log_interval_ms: u32,  // 数据记录间隔: 1000ms
}
```

### 报警级别
- **Info**: 信息提示（蓝色）
- **Warning**: 警告（黄色）
- **Critical**: 严重（红色）
- **Emergency**: 紧急（紫色）

## 功能特性

### 1. 触摸操作
- **导航按钮**: 页面切换
- **数据确认**: 报警确认
- **参数调整**: 设置修改
- **图表交互**: 趋势查看

### 2. 数据可视化
- **实时数值**: 数字显示当前值
- **状态指示**: 颜色编码数据质量
- **趋势图表**: 历史数据曲线
- **报警状态**: 视觉报警提示

### 3. 报警管理
```rust
// 报警检查示例
fn check_alarms(&mut self) -> Result<(), &'static str> {
    for data_point in &self.data_points {
        // 温度报警
        if data_point.tag.as_str() == "TEMP_01" && data_point.value > 30.0 {
            self.add_alarm(1, AlarmLevel::Warning, "High Temperature")?;
        }
        
        // 压力报警
        if data_point.tag.as_str() == "PRESS_01" && data_point.value > 1.5 {
            self.add_alarm(2, AlarmLevel::Critical, "High Pressure")?;
        }
    }
    Ok(())
}
```

### 4. 趋势分析
- **数据采集**: 自动记录历史数据
- **图表绘制**: 实时趋势曲线
- **统计分析**: 最大最小值计算
- **数据压缩**: 历史数据管理

## 扩展功能

### 1. 网络通信
```toml
[features]
ethernet = ["smoltcp", "stm32-eth"]
web-interface = ["ethernet"]
```

### 2. 数据记录
```toml
[features]
data-logging = ["embedded-storage"]
```

### 3. 音频报警
```toml
[features]
audio-alarm = ["pwm-pca9685"]
```

### 4. 安全功能
```toml
[features]
security = ["aes", "sha2"]
```

## 调试和测试

### 1. 显示测试
```bash
cargo run --bin display_test
```

### 2. 触摸测试
```bash
cargo run --bin touch_test
```

### 3. Modbus HMI测试
```bash
cargo run --bin modbus_hmi_test
```

### 4. Web HMI测试
```bash
cargo run --bin web_hmi_test
```

### 5. 报警测试
```bash
cargo run --bin alarm_test
```

## 性能优化

### 1. 显示优化
- **局部刷新**: 只更新变化区域
- **缓冲管理**: 双缓冲减少闪烁
- **字体优化**: 使用位图字体

### 2. 触摸优化
- **防抖处理**: 消除触摸抖动
- **响应时间**: 快速触摸响应
- **精度校准**: 触摸坐标校准

### 3. 数据优化
- **内存管理**: 循环缓冲区
- **计算优化**: 定点数运算
- **存储优化**: 数据压缩

## 注意事项

### 1. 硬件要求
- STM32F407VG或更高性能MCU
- 至少128KB RAM用于图形缓冲
- SPI接口支持高速通信

### 2. 软件限制
- 最大32个数据点
- 最大16个活动报警
- 最大8个趋势图表
- 每个趋势最多100个数据点

### 3. 实时性能
- 显示更新频率: 10Hz
- 触摸响应时间: <50ms
- 数据采集周期: 100ms
- 报警检查周期: 100ms

## 故障排除

### 1. 显示问题
```rust
// 检查SPI连接
// 验证显示初始化
// 检查电源电压
```

### 2. 触摸问题
```rust
// 检查触摸中断
// 校准触摸坐标
// 验证触摸阈值
```

### 3. 数据问题
```rust
// 检查数据源连接
// 验证数据格式
// 检查通信协议
```

### 4. 性能问题
```rust
// 监控CPU使用率
// 检查内存使用
// 优化更新频率
```

## 开发指南

### 1. 添加新页面
```rust
// 1. 在ScreenPage枚举中添加新页面
// 2. 实现页面绘制函数
// 3. 添加触摸处理函数
// 4. 更新导航逻辑
```

### 2. 添加新数据点
```rust
// 1. 在initialize_data_points中添加
// 2. 在update_data中添加更新逻辑
// 3. 在check_alarms中添加报警检查
```

### 3. 自定义报警
```rust
// 1. 定义报警条件
// 2. 设置报警级别
// 3. 配置报警消息
// 4. 实现报警处理
```

这个工业HMI系统提供了完整的人机界面解决方案，适用于各种工业自动化应用场景。