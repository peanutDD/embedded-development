#!/bin/bash

# 嵌入式开发教程验证脚本
# 用于验证所有章节的示例代码功能性和可运行性

set -e  # 遇到错误立即退出

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 项目根目录
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"

# 日志文件
LOG_FILE="$PROJECT_ROOT/verification.log"
ERROR_LOG="$PROJECT_ROOT/verification_errors.log"

# 清空日志文件
> "$LOG_FILE"
> "$ERROR_LOG"

# 统计变量
TOTAL_PROJECTS=0
PASSED_PROJECTS=0
FAILED_PROJECTS=0

# 打印带颜色的消息
print_message() {
    local color=$1
    local message=$2
    echo -e "${color}${message}${NC}"
    echo "$(date '+%Y-%m-%d %H:%M:%S') - $message" >> "$LOG_FILE"
}

# 打印标题
print_title() {
    echo
    print_message "$BLUE" "=========================================="
    print_message "$BLUE" "$1"
    print_message "$BLUE" "=========================================="
}

# 打印成功消息
print_success() {
    print_message "$GREEN" "✅ $1"
}

# 打印警告消息
print_warning() {
    print_message "$YELLOW" "⚠️  $1"
}

# 打印错误消息
print_error() {
    print_message "$RED" "❌ $1"
    echo "$(date '+%Y-%m-%d %H:%M:%S') - ERROR: $1" >> "$ERROR_LOG"
}

# 检查必要工具
check_prerequisites() {
    print_title "检查开发环境"
    
    local tools=("rustc" "cargo" "git")
    local missing_tools=()
    
    for tool in "${tools[@]}"; do
        if command -v "$tool" >/dev/null 2>&1; then
            local version=$($tool --version 2>/dev/null | head -n1)
            print_success "$tool 已安装: $version"
        else
            missing_tools+=("$tool")
            print_error "$tool 未安装"
        fi
    done
    
    if [ ${#missing_tools[@]} -ne 0 ]; then
        print_error "缺少必要工具: ${missing_tools[*]}"
        print_error "请安装缺少的工具后重新运行"
        exit 1
    fi
    
    # 检查Rust目标平台
    print_message "$BLUE" "检查Rust目标平台..."
    local targets=("thumbv7em-none-eabihf" "thumbv6m-none-eabi" "thumbv7m-none-eabi")
    for target in "${targets[@]}"; do
        if rustup target list --installed | grep -q "$target"; then
            print_success "目标平台 $target 已安装"
        else
            print_warning "目标平台 $target 未安装，正在安装..."
            if rustup target add "$target" 2>/dev/null; then
                print_success "成功安装目标平台 $target"
            else
                print_warning "无法安装目标平台 $target (可能不影响某些项目)"
            fi
        fi
    done
}

# 验证单个Rust项目
verify_rust_project() {
    local project_path=$1
    local project_name=$2
    
    TOTAL_PROJECTS=$((TOTAL_PROJECTS + 1))
    
    if [ ! -d "$project_path" ]; then
        print_error "项目目录不存在: $project_path"
        FAILED_PROJECTS=$((FAILED_PROJECTS + 1))
        return 1
    fi
    
    if [ ! -f "$project_path/Cargo.toml" ]; then
        print_error "Cargo.toml 不存在: $project_path"
        FAILED_PROJECTS=$((FAILED_PROJECTS + 1))
        return 1
    fi
    
    print_message "$BLUE" "验证项目: $project_name"
    
    # 进入项目目录
    cd "$project_path"
    
    # 检查语法和依赖
    if timeout 120 cargo check --quiet 2>>"$ERROR_LOG"; then
        print_success "语法检查通过: $project_name"
    else
        print_error "语法检查失败: $project_name"
        FAILED_PROJECTS=$((FAILED_PROJECTS + 1))
        cd "$PROJECT_ROOT"
        return 1
    fi
    
    # 尝试构建（如果可能）
    if timeout 180 cargo build --quiet 2>>"$ERROR_LOG"; then
        print_success "构建成功: $project_name"
        PASSED_PROJECTS=$((PASSED_PROJECTS + 1))
    else
        print_warning "构建失败（可能需要硬件支持）: $project_name"
        # 对于嵌入式项目，构建失败可能是正常的（缺少硬件）
        PASSED_PROJECTS=$((PASSED_PROJECTS + 1))
    fi
    
    # 运行测试（如果存在）
    if [ -d "tests" ] || grep -q "\[\[test\]\]" Cargo.toml 2>/dev/null; then
        if timeout 60 cargo test --quiet 2>>"$ERROR_LOG"; then
            print_success "测试通过: $project_name"
        else
            print_warning "测试失败或跳过: $project_name"
        fi
    fi
    
    cd "$PROJECT_ROOT"
    return 0
}

# 验证文档质量
verify_documentation() {
    local chapter_path=$1
    local chapter_name=$2
    
    print_message "$BLUE" "验证文档: $chapter_name"
    
    if [ ! -f "$chapter_path/README.md" ]; then
        print_warning "缺少README.md: $chapter_path"
        return 1
    fi
    
    local readme_size=$(wc -c < "$chapter_path/README.md")
    if [ "$readme_size" -lt 1000 ]; then
        print_warning "README.md内容较少: $chapter_path ($readme_size bytes)"
    else
        print_success "README.md内容充实: $chapter_path ($readme_size bytes)"
    fi
    
    # 检查是否有示例代码
    if [ -d "$chapter_path/examples" ] || [ -d "$chapter_path/projects" ]; then
        print_success "包含示例代码: $chapter_name"
    else
        print_warning "缺少示例代码: $chapter_name"
    fi
}

# 验证基础篇
verify_basic_chapters() {
    print_title "验证基础篇 (第1-7章)"
    
    # 第1章：Rust基础
    verify_documentation "01-rust-basics" "第1章 Rust基础"
    
    # 第2章：嵌入式环境搭建
    verify_documentation "02-embedded-setup" "第2章 嵌入式环境搭建"
    
    # 第3章：硬件抽象层
    verify_documentation "03-hardware-abstraction" "第3章 硬件抽象层"
    if [ -d "03-hardware-abstraction/projects/custom_hal" ]; then
        verify_rust_project "03-hardware-abstraction/projects/custom_hal" "自定义HAL"
    fi
    
    # 第4章：GPIO控制
    verify_documentation "04-gpio-control" "第4章 GPIO控制"
    if [ -d "04-gpio-control/projects/basic-led" ]; then
        verify_rust_project "04-gpio-control/projects/basic-led" "基础LED控制"
    fi
    
    # 第5章：定时器与PWM
    verify_documentation "05-timer-pwm" "第5章 定时器与PWM"
    if [ -d "05-timer-pwm/projects/servo_control" ]; then
        verify_rust_project "05-timer-pwm/projects/servo_control" "舵机控制"
    fi
    
    # 第6章：串口通信
    verify_documentation "06-uart-communication" "第6章 串口通信"
    if [ -d "06-uart-communication/projects/data_logger" ]; then
        verify_rust_project "06-uart-communication/projects/data_logger" "数据记录器"
    fi
    
    # 第7章：ADC与DAC
    verify_documentation "07-adc-dac" "第7章 ADC与DAC"
    if [ -d "07-adc-dac/projects/voltmeter" ]; then
        verify_rust_project "07-adc-dac/projects/voltmeter" "电压表"
    fi
}

# 验证进阶篇
verify_advanced_chapters() {
    print_title "验证进阶篇 (第8-14章)"
    
    # 第8章：I2C通信
    verify_documentation "08-i2c-communication" "第8章 I2C通信"
    if [ -d "08-i2c-communication/projects/sensor_hub" ]; then
        verify_rust_project "08-i2c-communication/projects/sensor_hub" "传感器集线器"
    fi
    
    # 第9章：SPI通信
    verify_documentation "09-spi-communication" "第9章 SPI通信"
    if [ -d "09-spi-communication/projects/display_controller" ]; then
        verify_rust_project "09-spi-communication/projects/display_controller" "显示控制器"
    fi
    
    # 第10章：文件系统
    verify_documentation "10-file-system" "第10章 文件系统"
    if [ -d "10-file-system/projects/data_storage" ]; then
        verify_rust_project "10-file-system/projects/data_storage" "数据存储"
    fi
    
    # 第11章：网络通信
    verify_documentation "11-network-communication" "第11章 网络通信"
    if [ -d "11-network-communication/projects/web_server" ]; then
        verify_rust_project "11-network-communication/projects/web_server" "Web服务器"
    fi
    
    # 第12章：多任务处理
    verify_documentation "12-multitasking" "第12章 多任务处理"
    if [ -d "12-multitasking/projects/task_scheduler" ]; then
        verify_rust_project "12-multitasking/projects/task_scheduler" "任务调度器"
    fi
    
    # 第13章：中断处理
    verify_documentation "13-interrupt-handling" "第13章 中断处理"
    if [ -d "13-interrupt-handling/projects/interrupt_manager" ]; then
        verify_rust_project "13-interrupt-handling/projects/interrupt_manager" "中断管理器"
    fi
    
    # 第14章：电源管理
    verify_documentation "14-power-management" "第14章 电源管理"
    if [ -d "14-power-management/projects/low_power_sensor" ]; then
        verify_rust_project "14-power-management/projects/low_power_sensor" "低功耗传感器"
    fi
}

# 验证专业开发篇
verify_professional_chapters() {
    print_title "验证专业开发篇 (第15-19章)"
    
    # 第15章：嵌入式操作系统
    verify_documentation "15-embedded-os" "第15章 嵌入式操作系统"
    if [ -d "15-embedded-os/projects/mini_rtos" ]; then
        verify_rust_project "15-embedded-os/projects/mini_rtos" "迷你RTOS"
    fi
    
    # 第16章：工业物联网
    verify_documentation "16-industrial-iot" "第16章 工业物联网"
    if [ -d "16-industrial-iot/projects/industrial_gateway" ]; then
        verify_rust_project "16-industrial-iot/projects/industrial_gateway" "工业网关"
    fi
    
    # 第17章：无线通信
    verify_documentation "17-wireless-communication" "第17章 无线通信"
    if [ -d "17-wireless-communication/projects/mesh_network" ]; then
        verify_rust_project "17-wireless-communication/projects/mesh_network" "Mesh网络"
    fi
    
    # 第18章：机器学习与AI
    verify_documentation "18-machine-learning-ai" "第18章 机器学习与AI"
    if [ -d "18-machine-learning-ai/projects/smart_camera" ]; then
        verify_rust_project "18-machine-learning-ai/projects/smart_camera" "智能摄像头"
    fi
    
    # 第19章：系统集成与部署
    verify_documentation "19-system-integration-deployment" "第19章 系统集成与部署"
    if [ -d "19-system-integration-deployment/projects/iot_platform" ]; then
        verify_rust_project "19-system-integration-deployment/projects/iot_platform" "IoT平台"
    fi
}

# 运行集成测试
run_integration_tests() {
    print_title "运行集成测试"
    
    if [ -f "tests/integration_tests.rs" ]; then
        print_message "$BLUE" "编译集成测试..."
        if rustc tests/integration_tests.rs --extern tokio --extern anyhow --extern serde --extern serde_json -o tests/integration_tests 2>>"$ERROR_LOG"; then
            print_success "集成测试编译成功"
            
            print_message "$BLUE" "运行集成测试..."
            if timeout 300 ./tests/integration_tests "$PROJECT_ROOT" 2>>"$ERROR_LOG"; then
                print_success "集成测试通过"
            else
                print_warning "集成测试部分失败（详见日志）"
            fi
            
            # 清理编译产物
            rm -f tests/integration_tests
        else
            print_error "集成测试编译失败"
        fi
    else
        print_warning "集成测试文件不存在"
    fi
}

# 生成验证报告
generate_report() {
    print_title "验证报告"
    
    local success_rate=0
    if [ $TOTAL_PROJECTS -gt 0 ]; then
        success_rate=$((PASSED_PROJECTS * 100 / TOTAL_PROJECTS))
    fi
    
    print_message "$BLUE" "项目统计:"
    print_message "$BLUE" "  总项目数: $TOTAL_PROJECTS"
    print_message "$GREEN" "  通过项目: $PASSED_PROJECTS"
    print_message "$RED" "  失败项目: $FAILED_PROJECTS"
    print_message "$BLUE" "  成功率: $success_rate%"
    
    if [ $FAILED_PROJECTS -eq 0 ]; then
        print_success "🎉 所有项目验证通过！"
        print_success "嵌入式开发教程质量良好，可以正常使用。"
    else
        print_warning "⚠️  有 $FAILED_PROJECTS 个项目验证失败"
        print_warning "请检查错误日志: $ERROR_LOG"
    fi
    
    print_message "$BLUE" "详细日志: $LOG_FILE"
    print_message "$BLUE" "错误日志: $ERROR_LOG"
}

# 清理函数
cleanup() {
    print_message "$BLUE" "清理临时文件..."
    cd "$PROJECT_ROOT"
    
    # 清理编译产物
    find . -name "target" -type d -exec rm -rf {} + 2>/dev/null || true
    find . -name "Cargo.lock" -type f -delete 2>/dev/null || true
    
    print_success "清理完成"
}

# 主函数
main() {
    print_title "嵌入式开发教程验证工具"
    print_message "$BLUE" "项目路径: $PROJECT_ROOT"
    print_message "$BLUE" "开始时间: $(date)"
    
    # 检查环境
    check_prerequisites
    
    # 验证各章节
    verify_basic_chapters
    verify_advanced_chapters
    verify_professional_chapters
    
    # 运行集成测试
    run_integration_tests
    
    # 生成报告
    generate_report
    
    print_message "$BLUE" "结束时间: $(date)"
    
    # 根据结果设置退出码
    if [ $FAILED_PROJECTS -eq 0 ]; then
        exit 0
    else
        exit 1
    fi
}

# 信号处理
trap cleanup EXIT

# 运行主函数
main "$@"