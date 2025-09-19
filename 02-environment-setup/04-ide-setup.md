# IDE和编辑器配置

## 概述

选择合适的IDE或编辑器对嵌入式开发效率至关重要。本章节将详细介绍如何配置主流开发环境，包括VS Code、CLion、Vim/Neovim等，以及推荐的插件和扩展。

## IDE选择指南

### 编辑器对比

| IDE/编辑器 | 优势 | 劣势 | 适合人群 |
|------------|------|------|----------|
| VS Code | 免费、插件丰富、轻量 | 功能相对简单 | 初学者、通用开发 |
| CLion | 强大调试、智能补全 | 付费、资源占用大 | 专业开发者 |
| Vim/Neovim | 高效、可定制性强 | 学习曲线陡峭 | 高级用户 |
| Emacs | 极度可定制 | 配置复杂 | 资深开发者 |
| Eclipse CDT | 功能完整、免费 | 界面老旧、较重 | 传统C/C++开发者 |

## VS Code 配置

### 基础安装

```bash
# 下载并安装VS Code
# 访问 https://code.visualstudio.com/

# 或使用包管理器
# macOS
brew install --cask visual-studio-code

# Ubuntu/Debian
sudo snap install code --classic
```

### 必备扩展

#### 1. Rust语言支持

```json
{
    "recommendations": [
        "rust-lang.rust-analyzer",
        "vadimcn.vscode-lldb",
        "serayuzgur.crates",
        "tamasfe.even-better-toml"
    ]
}
```

#### 2. 嵌入式开发扩展

```json
{
    "recommendations": [
        "marus25.cortex-debug",
        "ms-vscode.hexeditor",
        "webfreak.debug",
        "ms-vscode.cmake-tools",
        "twxs.cmake"
    ]
}
```

#### 3. 通用开发扩展

```json
{
    "recommendations": [
        "ms-vscode.vscode-json",
        "redhat.vscode-yaml",
        "ms-python.python",
        "ms-vscode.makefile-tools",
        "eamodio.gitlens"
    ]
}
```

### VS Code 配置文件

#### 1. 工作区配置 (.vscode/settings.json)

```json
{
    "rust-analyzer.checkOnSave.command": "clippy",
    "rust-analyzer.checkOnSave.extraArgs": ["--target", "thumbv7em-none-eabihf"],
    "rust-analyzer.cargo.target": "thumbv7em-none-eabihf",
    "rust-analyzer.cargo.features": ["stm32f411"],
    
    "files.associations": {
        "*.x": "ld",
        "memory.x": "ld",
        "*.ld": "ld"
    },
    
    "cortex-debug.enableTelemetry": false,
    "cortex-debug.showRTOS": true,
    
    "editor.formatOnSave": true,
    "editor.rulers": [100],
    "editor.tabSize": 4,
    
    "terminal.integrated.env.osx": {
        "PATH": "${env:PATH}:${workspaceFolder}/.cargo/bin"
    }
}
```

#### 2. 任务配置 (.vscode/tasks.json)

```json
{
    "version": "2.0.0",
    "tasks": [
        {
            "label": "cargo build",
            "type": "cargo",
            "command": "build",
            "args": ["--target", "thumbv7em-none-eabihf"],
            "group": {
                "kind": "build",
                "isDefault": true
            },
            "presentation": {
                "echo": true,
                "reveal": "always",
                "focus": false,
                "panel": "shared"
            },
            "problemMatcher": ["$rustc"]
        },
        {
            "label": "cargo check",
            "type": "cargo",
            "command": "check",
            "args": ["--target", "thumbv7em-none-eabihf"],
            "group": "build",
            "presentation": {
                "echo": true,
                "reveal": "silent",
                "focus": false,
                "panel": "shared"
            },
            "problemMatcher": ["$rustc"]
        },
        {
            "label": "cargo flash",
            "type": "shell",
            "command": "probe-rs",
            "args": [
                "run",
                "--chip", "STM32F411RETx",
                "target/thumbv7em-none-eabihf/debug/${workspaceFolderBasename}"
            ],
            "group": "build",
            "dependsOn": "cargo build",
            "presentation": {
                "echo": true,
                "reveal": "always",
                "focus": false,
                "panel": "shared"
            }
        }
    ]
}
```

#### 3. 调试配置 (.vscode/launch.json)

```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "Debug (probe-rs)",
            "type": "probe-rs-debug",
            "request": "launch",
            "chip": "STM32F411RETx",
            "flashingConfig": {
                "flashingEnabled": true,
                "resetAfterFlashing": true,
                "haltAfterReset": true
            },
            "cwd": "${workspaceFolder}",
            "program": "${workspaceFolder}/target/thumbv7em-none-eabihf/debug/${workspaceFolderBasename}",
            "rttConfig": {
                "enabled": true,
                "address": "auto",
                "decoders": [
                    {
                        "port": 0,
                        "timestamp": true,
                        "showLocation": true
                    }
                ]
            }
        },
        {
            "name": "Debug (OpenOCD)",
            "type": "cortex-debug",
            "request": "launch",
            "servertype": "openocd",
            "cwd": "${workspaceFolder}",
            "executable": "${workspaceFolder}/target/thumbv7em-none-eabihf/debug/${workspaceFolderBasename}",
            "configFiles": [
                "interface/stlink.cfg",
                "target/stm32f4x.cfg"
            ],
            "svdFile": "${workspaceFolder}/STM32F411.svd",
            "runToMain": true,
            "showDevDebugOutput": false
        }
    ]
}
```

### VS Code 使用技巧

#### 1. 快捷键配置

```json
// keybindings.json
[
    {
        "key": "ctrl+shift+b",
        "command": "workbench.action.tasks.runTask",
        "args": "cargo build"
    },
    {
        "key": "ctrl+shift+f",
        "command": "workbench.action.tasks.runTask",
        "args": "cargo flash"
    },
    {
        "key": "f5",
        "command": "workbench.action.debug.start"
    }
]
```

#### 2. 代码片段

```json
// rust.json (snippets)
{
    "Embedded main function": {
        "prefix": "main_embedded",
        "body": [
            "#![no_std]",
            "#![no_main]",
            "",
            "use panic_halt as _;",
            "use cortex_m_rt::entry;",
            "",
            "#[entry]",
            "fn main() -> ! {",
            "    $0",
            "    loop {}",
            "}"
        ],
        "description": "Embedded Rust main function template"
    }
}
```

## CLion 配置

### 安装和许可

```bash
# 下载CLion
# 访问 https://www.jetbrains.com/clion/

# 学生可申请免费许可证
# 访问 https://www.jetbrains.com/student/
```

### Rust插件配置

#### 1. 安装插件

- **Rust**: 官方Rust支持
- **TOML**: TOML文件支持
- **Serial Port Monitor**: 串口监控

#### 2. 项目配置

```cmake
# CMakeLists.txt (用于CLion项目结构)
cmake_minimum_required(VERSION 3.16)
project(embedded_rust)

# 设置Rust工具链
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# 添加Rust目标
add_custom_target(
    rust_build
    COMMAND cargo build --target thumbv7em-none-eabihf
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)

add_custom_target(
    rust_flash
    COMMAND probe-rs run --chip STM32F411RETx target/thumbv7em-none-eabihf/debug/my-project
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    DEPENDS rust_build
)
```

#### 3. 运行配置

```xml
<!-- .idea/runConfigurations/Flash.xml -->
<component name="ProjectRunConfigurationManager">
  <configuration default="false" name="Flash" type="CargoCommandRunConfiguration" factoryName="Cargo Command">
    <option name="command" value="run --target thumbv7em-none-eabihf" />
    <option name="workingDirectory" value="file://$PROJECT_DIR$" />
    <option name="channel" value="DEFAULT" />
    <option name="requiredFeatures" value="true" />
    <option name="allFeatures" value="false" />
    <option name="emulateTerminal" value="false" />
    <option name="withSudo" value="false" />
    <option name="buildTarget" value="REMOTE" />
    <option name="backtrace" value="SHORT" />
    <envs />
    <option name="isRedirectInput" value="false" />
    <option name="redirectInputPath" value="" />
    <method v="2">
      <option name="CARGO.BUILD_TASK_PROVIDER" enabled="true" />
    </method>
  </configuration>
</component>
```

### CLion 调试配置

#### 1. GDB调试器配置

```xml
<!-- .idea/runConfigurations/Debug.xml -->
<component name="ProjectRunConfigurationManager">
  <configuration default="false" name="Debug" type="GDBRemoteRunConfiguration" factoryName="GDB Remote Debug">
    <option name="LOCAL_EXECUTABLE_PATH" value="$PROJECT_DIR$/target/thumbv7em-none-eabihf/debug/my-project" />
    <option name="REMOTE_CONNECTION_TYPE" value="tcp" />
    <option name="REMOTE_HOST" value="localhost" />
    <option name="REMOTE_PORT" value="3333" />
    <option name="REMOTE_EXECUTABLE_PATH" value="" />
    <option name="GDB_PATH" value="arm-none-eabi-gdb" />
    <option name="DOWNLOAD_EXECUTABLE" value="true" />
    <method v="2" />
  </configuration>
</component>
```

## Vim/Neovim 配置

### 基础配置

#### 1. 安装Neovim

```bash
# macOS
brew install neovim

# Ubuntu/Debian
sudo apt-get install neovim

# 或从源码编译最新版本
git clone https://github.com/neovim/neovim.git
cd neovim
make CMAKE_BUILD_TYPE=Release
sudo make install
```

#### 2. 插件管理器

```lua
-- init.lua (使用lazy.nvim)
local lazypath = vim.fn.stdpath("data") .. "/lazy/lazy.nvim"
if not vim.loop.fs_stat(lazypath) then
  vim.fn.system({
    "git",
    "clone",
    "--filter=blob:none",
    "https://github.com/folke/lazy.nvim.git",
    "--branch=stable",
    lazypath,
  })
end
vim.opt.rtp:prepend(lazypath)

require("lazy").setup({
  -- Rust支持
  {
    "simrat39/rust-tools.nvim",
    dependencies = {
      "neovim/nvim-lspconfig",
      "nvim-lua/plenary.nvim",
      "mfussenegger/nvim-dap",
    },
  },
  
  -- LSP配置
  {
    "neovim/nvim-lspconfig",
    config = function()
      require("lspconfig").rust_analyzer.setup({
        settings = {
          ["rust-analyzer"] = {
            cargo = {
              target = "thumbv7em-none-eabihf",
            },
            checkOnSave = {
              command = "clippy",
              extraArgs = { "--target", "thumbv7em-none-eabihf" },
            },
          },
        },
      })
    end,
  },
  
  -- 自动补全
  {
    "hrsh7th/nvim-cmp",
    dependencies = {
      "hrsh7th/cmp-nvim-lsp",
      "hrsh7th/cmp-buffer",
      "hrsh7th/cmp-path",
      "L3MON4D3/LuaSnip",
    },
  },
  
  -- 文件树
  "nvim-tree/nvim-tree.lua",
  
  -- 模糊查找
  {
    "nvim-telescope/telescope.nvim",
    dependencies = { "nvim-lua/plenary.nvim" },
  },
  
  -- Git集成
  "lewis6991/gitsigns.nvim",
  
  -- 状态栏
  "nvim-lualine/lualine.nvim",
})
```

#### 3. Rust特定配置

```lua
-- rust.lua
local rt = require("rust-tools")

rt.setup({
  server = {
    on_attach = function(_, bufnr)
      -- 快捷键绑定
      vim.keymap.set("n", "<C-space>", rt.hover_actions.hover_actions, { buffer = bufnr })
      vim.keymap.set("n", "<Leader>a", rt.code_action_group.code_action_group, { buffer = bufnr })
    end,
    settings = {
      ["rust-analyzer"] = {
        cargo = {
          target = "thumbv7em-none-eabihf",
        },
        checkOnSave = {
          command = "clippy",
          extraArgs = { "--target", "thumbv7em-none-eabihf" },
        },
      },
    },
  },
  tools = {
    hover_actions = {
      auto_focus = true,
    },
  },
})
```

#### 4. 调试配置

```lua
-- dap.lua
local dap = require("dap")

dap.adapters.gdb = {
  type = "executable",
  command = "arm-none-eabi-gdb",
  args = { "-i", "dap" }
}

dap.configurations.rust = {
  {
    name = "Debug with GDB",
    type = "gdb",
    request = "launch",
    program = function()
      return vim.fn.input('Path to executable: ', vim.fn.getcwd() .. '/target/thumbv7em-none-eabihf/debug/', 'file')
    end,
    cwd = "${workspaceFolder}",
    stopAtBeginningOfMainSubprogram = false,
    setupCommands = {
      {
        text = "target extended-remote localhost:3333",
        description = "Connect to OpenOCD",
        ignoreFailures = false
      },
      {
        text = "load",
        description = "Load program",
        ignoreFailures = false
      },
      {
        text = "monitor reset halt",
        description = "Reset and halt",
        ignoreFailures = false
      }
    }
  },
}
```

### Vim 使用技巧

#### 1. 快捷键配置

```lua
-- keymaps.lua
local opts = { noremap = true, silent = true }

-- 编译和运行
vim.keymap.set("n", "<leader>cb", ":!cargo build --target thumbv7em-none-eabihf<CR>", opts)
vim.keymap.set("n", "<leader>cf", ":!probe-rs run --chip STM32F411RETx target/thumbv7em-none-eabihf/debug/%:t:r<CR>", opts)
vim.keymap.set("n", "<leader>cc", ":!cargo check --target thumbv7em-none-eabihf<CR>", opts)

-- LSP快捷键
vim.keymap.set("n", "gd", vim.lsp.buf.definition, opts)
vim.keymap.set("n", "K", vim.lsp.buf.hover, opts)
vim.keymap.set("n", "<leader>rn", vim.lsp.buf.rename, opts)
vim.keymap.set("n", "<leader>ca", vim.lsp.buf.code_action, opts)
```

## 其他编辑器配置

### Emacs 配置

#### 1. 基础配置

```elisp
;; init.el
(require 'package)
(add-to-list 'package-archives '("melpa" . "https://melpa.org/packages/") t)
(package-initialize)

;; 安装必要包
(unless (package-installed-p 'use-package)
  (package-refresh-contents)
  (package-install 'use-package))

;; Rust支持
(use-package rust-mode
  :ensure t
  :config
  (setq rust-format-on-save t))

(use-package lsp-mode
  :ensure t
  :hook (rust-mode . lsp)
  :commands lsp
  :config
  (setq lsp-rust-analyzer-cargo-target "thumbv7em-none-eabihf"))

(use-package company
  :ensure t
  :hook (after-init . global-company-mode))
```

### Sublime Text 配置

#### 1. 包安装

```json
// Package Control.sublime-settings
{
    "installed_packages": [
        "Rust Enhanced",
        "LSP",
        "LSP-rust-analyzer",
        "Terminus",
        "GitGutter"
    ]
}
```

#### 2. 项目配置

```json
// project.sublime-project
{
    "folders": [
        {
            "path": "."
        }
    ],
    "settings": {
        "LSP": {
            "rust-analyzer": {
                "settings": {
                    "rust-analyzer.cargo.target": "thumbv7em-none-eabihf",
                    "rust-analyzer.checkOnSave.command": "clippy",
                    "rust-analyzer.checkOnSave.extraArgs": ["--target", "thumbv7em-none-eabihf"]
                }
            }
        }
    },
    "build_systems": [
        {
            "name": "Cargo Build",
            "cmd": ["cargo", "build", "--target", "thumbv7em-none-eabihf"],
            "working_dir": "$project_path",
            "shell": false
        }
    ]
}
```

## 通用配置技巧

### 1. 代码格式化

```toml
# rustfmt.toml
max_width = 100
hard_tabs = false
tab_spaces = 4
newline_style = "Unix"
use_small_heuristics = "Default"
reorder_imports = true
reorder_modules = true
remove_nested_parens = true
edition = "2021"
```

### 2. Clippy配置

```toml
# clippy.toml
avoid-breaking-exported-api = false
msrv = "1.70"

# 允许的lint
allow = [
    "clippy::module_name_repetitions",
    "clippy::cast_possible_truncation",
]

# 禁止的lint
deny = [
    "clippy::unwrap_used",
    "clippy::expect_used",
    "clippy::panic",
]
```

### 3. 编辑器通用配置

```ini
# .editorconfig
root = true

[*]
charset = utf-8
end_of_line = lf
insert_final_newline = true
trim_trailing_whitespace = true

[*.rs]
indent_style = space
indent_size = 4

[*.toml]
indent_style = space
indent_size = 2

[*.md]
trim_trailing_whitespace = false
```

## 调试集成

### 1. RTT集成

大多数现代IDE都支持RTT输出显示：

```rust
// 在代码中使用RTT
use rtt_target::{rprintln, rtt_init_print};

#[cortex_m_rt::entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("Debug output from IDE");
    
    loop {
        // 主循环
    }
}
```

### 2. 串口监控

```json
// VS Code中的串口监控任务
{
    "label": "Serial Monitor",
    "type": "shell",
    "command": "screen",
    "args": ["/dev/ttyUSB0", "115200"],
    "group": "build",
    "presentation": {
        "echo": true,
        "reveal": "always",
        "focus": false,
        "panel": "new"
    }
}
```

## 性能优化

### 1. IDE性能调优

```json
// VS Code性能优化
{
    "rust-analyzer.cargo.loadOutDirsFromCheck": true,
    "rust-analyzer.procMacro.enable": true,
    "rust-analyzer.checkOnSave.enable": true,
    "rust-analyzer.checkOnSave.allTargets": false,
    "files.watcherExclude": {
        "**/target/**": true,
        "**/.git/**": true
    }
}
```

### 2. 大项目优化

```toml
# Cargo.toml
[profile.dev]
incremental = true
debug = 1  # 减少调试信息以提高编译速度

[profile.dev.package."*"]
opt-level = 1  # 依赖包轻微优化
```

## 最佳实践

### 1. 工作区组织

```
embedded-workspace/
├── .vscode/
│   ├── settings.json
│   ├── tasks.json
│   └── launch.json
├── boards/
│   ├── stm32f4/
│   └── esp32/
├── common/
│   └── src/
├── tools/
│   └── scripts/
└── Cargo.toml
```

### 2. 版本控制

```gitignore
# .gitignore
/target/
**/*.rs.bk
Cargo.lock  # 对于库项目
.vscode/launch.json  # 包含敏感信息时
.idea/
*.swp
*.swo
*~
```

### 3. 团队协作

```json
// .vscode/extensions.json
{
    "recommendations": [
        "rust-lang.rust-analyzer",
        "vadimcn.vscode-lldb",
        "marus25.cortex-debug"
    ],
    "unwantedRecommendations": [
        "ms-vscode.cpptools"
    ]
}
```

## 故障排除

### 1. 常见问题

- **rust-analyzer无法找到目标**: 检查`.cargo/config.toml`中的目标配置
- **调试器连接失败**: 确认OpenOCD或probe-rs正在运行
- **代码补全不工作**: 重启LSP服务器或重新索引项目

### 2. 诊断命令

```bash
# 检查rust-analyzer状态
rust-analyzer --version

# 检查目标安装
rustup target list --installed

# 检查调试器连接
probe-rs list
```

## 下一步

配置好IDE后，建议：

1. 学习[项目模板](./05-project-templates.md)快速创建项目
2. 掌握[交叉编译](./06-cross-compilation.md)优化技巧
3. 实践调试和开发工作流程

---

**现在你拥有了高效的嵌入式开发环境！** ⚡