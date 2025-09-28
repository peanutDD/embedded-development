# Display Controller - 嵌入式显示控制器

一个功能完整的嵌入式显示控制器库，支持多种显示设备和丰富的图形功能。

## 🚀 特性

### 支持的显示设备
- **OLED显示屏** - SSD1306控制器
- **LCD显示屏** - ST7735控制器
- **电子纸显示** - Waveshare EPD系列

### 图形功能
- 基本图形绘制（点、线、矩形、圆形）
- 文本渲染和多字体支持
- 图像显示（BMP、TGA格式）
- 自定义渲染器
- 动画支持（可选）

### 通信接口
- I2C通信支持
- SPI通信支持
- 并行接口支持

## 📦 安装

在你的 `Cargo.toml` 中添加：

```toml
[dependencies]
display-controller = "0.1.0"

# 根据需要启用特定功能
display-controller = { version = "0.1.0", features = ["oled", "lcd", "advanced-graphics"] }
```

## 🔧 功能特性

- `oled` - OLED显示支持（默认启用）
- `lcd` - LCD显示支持（默认启用）
- `epaper` - 电子纸显示支持
- `advanced-graphics` - 高级图形功能（图像支持）
- `animation` - 动画支持
- `touch-support` - 触摸屏支持
- `async` - 异步操作支持
- `simulator` - 桌面模拟器支持（开发用）

## 🚀 快速开始

### OLED显示示例

```rust
use display_controller::displays::oled::OledDisplay;
use display_controller::graphics::renderer::Renderer;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Rectangle, PrimitiveStyle};
use embedded_graphics::pixelcolor::BinaryColor;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // 初始化I2C接口
    let i2c = /* 你的I2C实现 */;
    
    // 创建OLED显示器
    let mut display = OledDisplay::new(i2c)?;
    display.init()?;
    
    // 创建渲染器
    let mut renderer = Renderer::new(&mut display);
    
    // 绘制矩形
    let rect = Rectangle::new(Point::new(10, 10), Size::new(50, 30))
        .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1));
    
    renderer.draw(&rect)?;
    renderer.flush()?;
    
    Ok(())
}
```

### LCD显示示例

```rust
use display_controller::displays::lcd::LcdDisplay;
use display_controller::graphics::renderer::Renderer;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::Circle;
use embedded_graphics::pixelcolor::Rgb565;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // 初始化SPI接口
    let spi = /* 你的SPI实现 */;
    let dc_pin = /* 数据/命令引脚 */;
    let rst_pin = /* 复位引脚 */;
    
    // 创建LCD显示器
    let mut display = LcdDisplay::new(spi, dc_pin, rst_pin)?;
    display.init()?;
    
    // 创建渲染器
    let mut renderer = Renderer::new(&mut display);
    
    // 绘制彩色圆形
    let circle = Circle::new(Point::new(64, 64), 30)
        .into_styled(PrimitiveStyle::with_fill(Rgb565::RED));
    
    renderer.draw(&circle)?;
    renderer.flush()?;
    
    Ok(())
}
```

## 📁 项目结构

```
src/
├── main.rs              # 主程序入口
├── lib.rs               # 库入口
├── displays/            # 显示设备驱动
│   ├── mod.rs
│   ├── oled.rs          # OLED显示驱动
│   ├── lcd.rs           # LCD显示驱动
│   └── epaper.rs        # 电子纸显示驱动
└── graphics/            # 图形处理
    ├── mod.rs
    ├── renderer.rs      # 渲染器
    └── fonts.rs         # 字体管理
```

## 🔌 硬件连接

### OLED (I2C)
```
OLED    MCU
VCC  -> 3.3V
GND  -> GND
SCL  -> I2C_SCL
SDA  -> I2C_SDA
```

### LCD (SPI)
```
LCD     MCU
VCC  -> 3.3V
GND  -> GND
SCK  -> SPI_SCK
MOSI -> SPI_MOSI
CS   -> SPI_CS
DC   -> GPIO
RST  -> GPIO
```

## 🎯 示例程序

运行示例程序：

```bash
# OLED示例
cargo run --example oled_demo --features oled

# LCD示例
cargo run --example lcd_demo --features lcd

# 电子纸示例
cargo run --example epaper_demo --features epaper

# 图形功能示例
cargo run --example graphics_demo --features advanced-graphics

# 桌面模拟器
cargo run --example simulator --features simulator
```

## 🧪 测试

```bash
# 运行所有测试
cargo test

# 运行特定功能测试
cargo test --features oled,lcd

# 运行基准测试
cargo bench
```

## 📚 API文档

生成并查看完整的API文档：

```bash
cargo doc --open --all-features
```

## 🤝 贡献

欢迎贡献代码！请遵循以下步骤：

1. Fork 本仓库
2. 创建功能分支 (`git checkout -b feature/amazing-feature`)
3. 提交更改 (`git commit -m 'Add some amazing feature'`)
4. 推送到分支 (`git push origin feature/amazing-feature`)
5. 开启 Pull Request

## 📄 许可证

本项目采用 MIT 或 Apache-2.0 双重许可证。详见 [LICENSE-MIT](LICENSE-MIT) 和 [LICENSE-APACHE](LICENSE-APACHE) 文件。

## 🔗 相关链接

- [embedded-graphics](https://github.com/embedded-graphics/embedded-graphics) - 嵌入式图形库
- [embedded-hal](https://github.com/rust-embedded/embedded-hal) - 嵌入式硬件抽象层
- [SSD1306 驱动](https://github.com/jamwaffles/ssd1306) - OLED显示驱动
- [ST7735 驱动](https://github.com/jamwaffles/st7735-lcd-rs) - LCD显示驱动

## 📞 支持

如果你遇到问题或有建议，请：

- 查看 [Issues](https://github.com/yourusername/display-controller/issues)
- 创建新的 Issue
- 参与 [Discussions](https://github.com/yourusername/display-controller/discussions)

---

**Display Controller** - 让嵌入式显示开发更简单！ 🎨