#!/bin/bash

# Rust嵌入式开发环境检查脚本
# 用于验证开发环境是否正确配置

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 检查结果统计
TOTAL_CHECKS=0
PASSED_CHECKS=0
FAILED_CHECKS=0

# 打印带颜色的消息
print_status() {
    local status=$1
    local message=$2
    
    TOTAL_CHECKS=$((TOTAL_CHECKS + 1))
    
    if [ "$status" = "PASS" ]; then
        echo -e "${GREEN}✓${NC} $message"
        PASSED_CHECKS=$((PASSED_CHECKS + 1))
    elif [ "$status" = "FAIL" ]; then
        echo -e "${RED}✗${NC} $message"
        FAILED_CHECKS=$((FAILED_CHECKS + 1))
    elif [ "$status" = "WARN" ]; then
        echo -e "${YELLOW}⚠${NC} $message"
    else
        echo -e "${BLUE}ℹ${NC} $message"
    fi
}

print_header() {
    echo -e "\n${BLUE}=== $1 ===${NC}"
}

# 检查命令是否存在
check_command() {
    local cmd=$1
    local name=$2
    
    if command -v "$cmd" >/dev/null 2>&1; then
        local version=$($cmd --version 2>/dev/null | head -n1 || echo "未知版本")
        print_status "PASS" "$name 已安装: $version"
        return 0
    else
        print_status "FAIL" "$name 未安装"
        return 1
    fi
}

# 检查Rust工具链
check_rust_toolchain() {
    print_header "Rust工具链检查"
    
    # 检查rustup
    if check_command "rustup" "rustup"; then
        # 检查默认工具链
        local default_toolchain=$(rustup default 2>/dev/null || echo "未设置")
        print_status "INFO" "默认工具链: $default_toolchain"
        
        # 检查已安装的工具链
        echo -e "\n已安装的工具链:"
        rustup toolchain list 2>/dev/null | while read -r line; do
            echo "  $line"
        done
    fi
    
    # 检查cargo
    check_command "cargo" "Cargo"
    
    # 检查rustc
    if check_command "rustc" "Rust编译器"; then
        local rustc_version=$(rustc --version)
        local rustc_commit=$(echo "$rustc_version" | grep -o '[a-f0-9]\{9\}' | head -n1)
        print_status "INFO" "编译器提交: $rustc_commit"
    fi
}

# 检查目标架构支持
check_target_support() {
    print_header "目标架构支持检查"
    
    # 常用的嵌入式目标架构
    local targets=(
        "thumbv6m-none-eabi"
        "thumbv7m-none-eabi"
        "thumbv7em-none-eabi"
        "thumbv7em-none-eabihf"
        "riscv32i-unknown-none-elf"
        "riscv32imac-unknown-none-elf"
    )
    
    for target in "${targets[@]}"; do
        if rustup target list --installed 2>/dev/null | grep -q "^$target"; then
            print_status "PASS" "目标架构 $target 已安装"
        else
            print_status "WARN" "目标架构 $target 未安装 (可选)"
        fi
    done
}

# 检查调试工具
check_debug_tools() {
    print_header "调试工具检查"
    
    # 检查probe-rs
    if check_command "probe-rs" "probe-rs"; then
        # 检查支持的芯片
        local chip_count=$(probe-rs chip list 2>/dev/null | wc -l || echo "0")
        print_status "INFO" "支持的芯片数量: $chip_count"
    fi
    
    # 检查OpenOCD
    check_command "openocd" "OpenOCD"
    
    # 检查GDB
    if check_command "arm-none-eabi-gdb" "ARM GDB"; then
        :
    elif check_command "gdb-multiarch" "GDB Multiarch"; then
        :
    else
        print_status "WARN" "未找到合适的GDB调试器"
    fi
}

# 检查开发工具
check_development_tools() {
    print_header "开发工具检查"
    
    # 检查cargo扩展
    local cargo_extensions=(
        "cargo-generate"
        "cargo-flash"
        "cargo-embed"
        "cargo-binutils"
        "cargo-audit"
        "cargo-deny"
    )
    
    for ext in "${cargo_extensions[@]}"; do
        if check_command "$ext" "$ext"; then
            :
        else
            print_status "WARN" "$ext 未安装 (推荐安装)"
        fi
    done
    
    # 检查其他有用的工具
    check_command "git" "Git"
    check_command "make" "Make"
}

# 检查环境变量
check_environment_variables() {
    print_header "环境变量检查"
    
    # 检查PATH
    if echo "$PATH" | grep -q "\.cargo/bin"; then
        print_status "PASS" "Cargo bin目录在PATH中"
    else
        print_status "WARN" "Cargo bin目录不在PATH中"
    fi
    
    # 检查RUST_TARGET_PATH
    if [ -n "$RUST_TARGET_PATH" ]; then
        print_status "INFO" "RUST_TARGET_PATH: $RUST_TARGET_PATH"
    else
        print_status "INFO" "RUST_TARGET_PATH 未设置 (通常不需要)"
    fi
    
    # 检查其他相关环境变量
    local env_vars=("RUSTUP_HOME" "CARGO_HOME" "RUSTFLAGS")
    for var in "${env_vars[@]}"; do
        if [ -n "${!var}" ]; then
            print_status "INFO" "$var: ${!var}"
        fi
    done
}

# 检查项目配置
check_project_configuration() {
    print_header "项目配置检查"
    
    # 检查.cargo/config.toml
    if [ -f ".cargo/config.toml" ]; then
        print_status "PASS" "找到 .cargo/config.toml"
        
        # 检查目标配置
        if grep -q "target.*=" ".cargo/config.toml" 2>/dev/null; then
            local target=$(grep "target.*=" ".cargo/config.toml" | head -n1 | cut -d'"' -f2)
            print_status "INFO" "默认目标: $target"
        fi
        
        # 检查runner配置
        if grep -q "runner.*=" ".cargo/config.toml" 2>/dev/null; then
            print_status "PASS" "配置了runner"
        else
            print_status "WARN" "未配置runner"
        fi
    else
        print_status "WARN" "未找到 .cargo/config.toml"
    fi
    
    # 检查Cargo.toml
    if [ -f "Cargo.toml" ]; then
        print_status "PASS" "找到 Cargo.toml"
        
        # 检查嵌入式相关依赖
        local embedded_deps=("cortex-m" "embedded-hal" "nb")
        for dep in "${embedded_deps[@]}"; do
            if grep -q "^$dep.*=" "Cargo.toml" 2>/dev/null; then
                print_status "PASS" "依赖 $dep 已配置"
            fi
        done
    fi
}

# 执行构建测试
test_build() {
    print_header "构建测试"
    
    if [ -f "Cargo.toml" ]; then
        echo "尝试构建项目..."
        
        if cargo check --quiet 2>/dev/null; then
            print_status "PASS" "项目检查通过"
        else
            print_status "FAIL" "项目检查失败"
        fi
        
        if cargo build --quiet 2>/dev/null; then
            print_status "PASS" "项目构建成功"
        else
            print_status "FAIL" "项目构建失败"
        fi
    else
        print_status "WARN" "当前目录不是Rust项目"
    fi
}

# 生成诊断报告
generate_report() {
    print_header "诊断报告"
    
    echo "系统信息:"
    echo "  操作系统: $(uname -s)"
    echo "  架构: $(uname -m)"
    echo "  内核版本: $(uname -r)"
    
    if command -v lsb_release >/dev/null 2>&1; then
        echo "  发行版: $(lsb_release -d | cut -f2)"
    fi
    
    echo ""
    echo "检查统计:"
    echo "  总检查项: $TOTAL_CHECKS"
    echo "  通过: $PASSED_CHECKS"
    echo "  失败: $FAILED_CHECKS"
    echo "  成功率: $((PASSED_CHECKS * 100 / TOTAL_CHECKS))%"
}

# 提供修复建议
provide_suggestions() {
    print_header "修复建议"
    
    if [ $FAILED_CHECKS -gt 0 ]; then
        echo "发现问题，建议执行以下操作:"
        echo ""
        
        if ! command -v rustup >/dev/null 2>&1; then
            echo "1. 安装Rust工具链:"
            echo "   curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh"
            echo ""
        fi
        
        if ! command -v probe-rs >/dev/null 2>&1; then
            echo "2. 安装probe-rs:"
            echo "   cargo install probe-rs --features cli"
            echo ""
        fi
        
        echo "3. 安装常用目标架构:"
        echo "   rustup target add thumbv7em-none-eabihf"
        echo "   rustup target add thumbv6m-none-eabi"
        echo ""
        
        echo "4. 安装推荐的cargo扩展:"
        echo "   cargo install cargo-generate cargo-flash cargo-embed"
        echo ""
    else
        echo "✓ 环境配置良好，可以开始嵌入式开发！"
    fi
}

# 主函数
main() {
    echo -e "${BLUE}"
    echo "========================================"
    echo "  Rust嵌入式开发环境检查工具"
    echo "========================================"
    echo -e "${NC}"
    
    check_rust_toolchain
    check_target_support
    check_debug_tools
    check_development_tools
    check_environment_variables
    check_project_configuration
    test_build
    
    echo ""
    generate_report
    echo ""
    provide_suggestions
    
    # 返回适当的退出码
    if [ $FAILED_CHECKS -eq 0 ]; then
        exit 0
    else
        exit 1
    fi
}

# 处理命令行参数
case "${1:-}" in
    --help|-h)
        echo "用法: $0 [选项]"
        echo ""
        echo "选项:"
        echo "  --help, -h     显示此帮助信息"
        echo "  --version, -v  显示版本信息"
        echo ""
        echo "此脚本检查Rust嵌入式开发环境的配置状态。"
        exit 0
        ;;
    --version|-v)
        echo "Rust嵌入式环境检查工具 v1.0.0"
        exit 0
        ;;
    *)
        main "$@"
        ;;
esac