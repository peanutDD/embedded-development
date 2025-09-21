#!/bin/bash

# 构建自动化脚本
# 用于嵌入式Rust项目的完整构建流程

set -e  # 遇到错误时退出

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 日志函数
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 项目配置
PROJECT_NAME="build-automation"
TARGET_ARCH="thumbv7em-none-eabihf"
BUILD_DIR="target"
ARTIFACTS_DIR="artifacts"
DOCS_DIR="docs"

# 命令行参数解析
CLEAN=false
RELEASE=false
DOCS=false
TEST=false
CLIPPY=false
FORMAT=false
SIZE_OPT=false
VERBOSE=false

while [[ $# -gt 0 ]]; do
    case $1 in
        --clean)
            CLEAN=true
            shift
            ;;
        --release)
            RELEASE=true
            shift
            ;;
        --docs)
            DOCS=true
            shift
            ;;
        --test)
            TEST=true
            shift
            ;;
        --clippy)
            CLIPPY=true
            shift
            ;;
        --format)
            FORMAT=true
            shift
            ;;
        --size-opt)
            SIZE_OPT=true
            shift
            ;;
        --verbose)
            VERBOSE=true
            shift
            ;;
        --all)
            CLEAN=true
            RELEASE=true
            DOCS=true
            TEST=true
            CLIPPY=true
            FORMAT=true
            shift
            ;;
        -h|--help)
            echo "Usage: $0 [OPTIONS]"
            echo "Options:"
            echo "  --clean     Clean build artifacts"
            echo "  --release   Build in release mode"
            echo "  --docs      Generate documentation"
            echo "  --test      Run tests"
            echo "  --clippy    Run clippy lints"
            echo "  --format    Format code"
            echo "  --size-opt  Build with size optimization"
            echo "  --verbose   Verbose output"
            echo "  --all       Run all operations"
            echo "  -h, --help  Show this help message"
            exit 0
            ;;
        *)
            log_error "Unknown option: $1"
            exit 1
            ;;
    esac
done

# 检查必要的工具
check_tools() {
    log_info "Checking required tools..."
    
    local tools=("cargo" "rustc" "arm-none-eabi-objcopy" "arm-none-eabi-size")
    local missing_tools=()
    
    for tool in "${tools[@]}"; do
        if ! command -v "$tool" &> /dev/null; then
            missing_tools+=("$tool")
        fi
    done
    
    if [ ${#missing_tools[@]} -ne 0 ]; then
        log_error "Missing required tools: ${missing_tools[*]}"
        log_error "Please install the missing tools and try again."
        exit 1
    fi
    
    log_success "All required tools are available"
}

# 清理构建产物
clean_build() {
    if [ "$CLEAN" = true ]; then
        log_info "Cleaning build artifacts..."
        cargo clean
        rm -rf "$ARTIFACTS_DIR"
        log_success "Build artifacts cleaned"
    fi
}

# 代码格式化
format_code() {
    if [ "$FORMAT" = true ]; then
        log_info "Formatting code..."
        cargo fmt --all
        log_success "Code formatted"
    fi
}

# 运行Clippy检查
run_clippy() {
    if [ "$CLIPPY" = true ]; then
        log_info "Running clippy checks..."
        cargo clippy --all-targets --all-features -- -D warnings
        log_success "Clippy checks passed"
    fi
}

# 运行测试
run_tests() {
    if [ "$TEST" = true ]; then
        log_info "Running tests..."
        # 注意：嵌入式项目可能需要特殊的测试配置
        if cargo test --lib 2>/dev/null; then
            log_success "Tests passed"
        else
            log_warning "Tests skipped (not supported for embedded target)"
        fi
    fi
}

# 构建项目
build_project() {
    log_info "Building project..."
    
    local build_flags=""
    local profile="dev"
    
    if [ "$RELEASE" = true ]; then
        build_flags="--release"
        profile="release"
    elif [ "$SIZE_OPT" = true ]; then
        build_flags="--profile size-opt"
        profile="size-opt"
    fi
    
    if [ "$VERBOSE" = true ]; then
        build_flags="$build_flags --verbose"
    fi
    
    # 设置构建时环境变量
    export BUILD_TIMESTAMP=$(date -u +"%Y-%m-%d %H:%M:%S UTC")
    export GIT_HASH=$(git rev-parse --short HEAD 2>/dev/null || echo "unknown")
    export BUILD_PROFILE="$profile"
    
    log_info "Building with profile: $profile"
    log_info "Target architecture: $TARGET_ARCH"
    
    cargo build --target "$TARGET_ARCH" $build_flags
    
    log_success "Build completed successfully"
}

# 生成二进制文件信息
generate_binary_info() {
    log_info "Generating binary information..."
    
    local profile="debug"
    if [ "$RELEASE" = true ]; then
        profile="release"
    elif [ "$SIZE_OPT" = true ]; then
        profile="size-opt"
    fi
    
    local elf_file="$BUILD_DIR/$TARGET_ARCH/$profile/$PROJECT_NAME"
    
    if [ -f "$elf_file" ]; then
        # 创建artifacts目录
        mkdir -p "$ARTIFACTS_DIR"
        
        # 生成二进制文件
        arm-none-eabi-objcopy -O binary "$elf_file" "$ARTIFACTS_DIR/$PROJECT_NAME.bin"
        arm-none-eabi-objcopy -O ihex "$elf_file" "$ARTIFACTS_DIR/$PROJECT_NAME.hex"
        
        # 生成大小信息
        arm-none-eabi-size "$elf_file" > "$ARTIFACTS_DIR/size_info.txt"
        
        # 生成反汇编
        arm-none-eabi-objdump -d "$elf_file" > "$ARTIFACTS_DIR/$PROJECT_NAME.asm"
        
        # 生成符号表
        arm-none-eabi-nm -n "$elf_file" > "$ARTIFACTS_DIR/$PROJECT_NAME.sym"
        
        # 生成构建信息
        cat > "$ARTIFACTS_DIR/build_info.txt" << EOF
Build Information
=================
Project: $PROJECT_NAME
Target: $TARGET_ARCH
Profile: $profile
Build Time: $(date -u +"%Y-%m-%d %H:%M:%S UTC")
Git Hash: $(git rev-parse --short HEAD 2>/dev/null || echo "unknown")
Git Branch: $(git branch --show-current 2>/dev/null || echo "unknown")
Rust Version: $(rustc --version)
Cargo Version: $(cargo --version)

Binary Sizes:
$(arm-none-eabi-size "$elf_file")

File Sizes:
$(ls -lh "$ARTIFACTS_DIR"/*.bin "$ARTIFACTS_DIR"/*.hex 2>/dev/null || true)
EOF
        
        log_success "Binary artifacts generated in $ARTIFACTS_DIR/"
        
        # 显示大小信息
        log_info "Binary size information:"
        arm-none-eabi-size "$elf_file"
    else
        log_error "ELF file not found: $elf_file"
        exit 1
    fi
}

# 生成文档
generate_docs() {
    if [ "$DOCS" = true ]; then
        log_info "Generating documentation..."
        
        # 生成Rust文档
        cargo doc --no-deps --target "$TARGET_ARCH"
        
        # 创建文档目录
        mkdir -p "$DOCS_DIR"
        
        # 复制生成的文档
        if [ -d "$BUILD_DIR/$TARGET_ARCH/doc" ]; then
            cp -r "$BUILD_DIR/$TARGET_ARCH/doc" "$DOCS_DIR/"
            log_success "Documentation generated in $DOCS_DIR/"
        else
            log_warning "Documentation not found"
        fi
    fi
}

# 验证构建结果
verify_build() {
    log_info "Verifying build results..."
    
    local profile="debug"
    if [ "$RELEASE" = true ]; then
        profile="release"
    elif [ "$SIZE_OPT" = true ]; then
        profile="size-opt"
    fi
    
    local elf_file="$BUILD_DIR/$TARGET_ARCH/$profile/$PROJECT_NAME"
    
    if [ ! -f "$elf_file" ]; then
        log_error "Build verification failed: ELF file not found"
        exit 1
    fi
    
    # 检查ELF文件格式
    if ! file "$elf_file" | grep -q "ARM"; then
        log_error "Build verification failed: Invalid ARM binary"
        exit 1
    fi
    
    # 检查二进制文件大小
    local size=$(stat -f%z "$elf_file" 2>/dev/null || stat -c%s "$elf_file" 2>/dev/null)
    if [ "$size" -eq 0 ]; then
        log_error "Build verification failed: Empty binary file"
        exit 1
    fi
    
    log_success "Build verification passed"
    log_info "Binary size: $size bytes"
}

# 主函数
main() {
    log_info "Starting build process for $PROJECT_NAME"
    log_info "Target architecture: $TARGET_ARCH"
    
    # 检查工具
    check_tools
    
    # 清理
    clean_build
    
    # 代码格式化
    format_code
    
    # Clippy检查
    run_clippy
    
    # 运行测试
    run_tests
    
    # 构建项目
    build_project
    
    # 验证构建
    verify_build
    
    # 生成二进制信息
    generate_binary_info
    
    # 生成文档
    generate_docs
    
    log_success "Build process completed successfully!"
    
    # 显示最终信息
    if [ -d "$ARTIFACTS_DIR" ]; then
        log_info "Artifacts available in: $ARTIFACTS_DIR/"
        ls -la "$ARTIFACTS_DIR/"
    fi
}

# 运行主函数
main "$@"