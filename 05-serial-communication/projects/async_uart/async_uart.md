async_uart/
├── 📄 核心文档
│   ├── README.md              # 项目说明和快速开始
│   ├── CHANGELOG.md           # 版本变更记录
│   ├── CONTRIBUTING.md        # 贡献指南
│   └── LICENSE               # MIT许可证
├── 🔧 配置文件
│   ├── Cargo.toml            # 完整项目配置
│   ├── Cargo_simple.toml     # 简化测试配置
│   └── .gitignore           # Git忽略规则
├── 💻 核心代码 (src/)
│   ├── lib.rs               # 库入口
│   ├── config.rs            # 配置管理
│   ├── error.rs             # 错误处理
│   ├── traits.rs            # 核心trait定义
│   ├── buffer/              # 缓冲区实现
│   ├── hal/                 # 硬件抽象层
│   └── protocol/            # 协议处理
├── 📚 示例程序 (examples/)
│   ├── basic_usage.rs       # 基本使用示例
│   ├── protocol_demo.rs     # 协议演示
│   ├── buffer_demo.rs       # 缓冲区演示
│   └── advanced_features.rs # 高级功能演示
├── 🧪 测试套件 (tests/)
│   ├── integration_tests.rs # 集成测试
│   └── stress_tests.rs      # 压力测试
├── ⚡ 性能测试 (benches/)
│   └── performance_benchmarks.rs # 性能基准测试
└── 📖 详细文档 (docs/)
    ├── user_guide.md        # 用户使用指南
    ├── api_reference.md     # API参考文档
    └── project_summary.md   # 项目技术总结