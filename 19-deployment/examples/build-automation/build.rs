use std::env;
use std::fs;
use std::path::Path;
use std::process::Command;

fn main() {
    // 获取构建时间
    let build_date = chrono::Utc::now().format("%Y-%m-%d %H:%M:%S UTC").to_string();
    println!("cargo:rustc-env=BUILD_DATE={}", build_date);

    // 获取Git哈希
    let git_hash = get_git_hash().unwrap_or_else(|| "unknown".to_string());
    println!("cargo:rustc-env=GIT_HASH={}", git_hash);

    // 获取目标架构
    let target = env::var("TARGET").unwrap_or_else(|_| "unknown".to_string());
    println!("cargo:rustc-env=TARGET={}", target);

    // 获取构建配置
    let profile = env::var("PROFILE").unwrap_or_else(|_| "debug".to_string());
    println!("cargo:rustc-env=PROFILE={}", profile);

    // 生成内存布局文件
    generate_memory_layout();

    // 生成链接器脚本
    generate_linker_script();

    // 生成版本信息
    generate_version_info();

    // 配置构建依赖
    configure_build_dependencies();

    // 设置重新构建条件
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=link.x");
    println!("cargo:rerun-if-changed=.git/HEAD");
    println!("cargo:rerun-if-changed=Cargo.toml");
}

fn get_git_hash() -> Option<String> {
    let output = Command::new("git")
        .args(&["rev-parse", "--short", "HEAD"])
        .output()
        .ok()?;

    if output.status.success() {
        let hash = String::from_utf8(output.stdout).ok()?;
        Some(hash.trim().to_string())
    } else {
        None
    }
}

fn generate_memory_layout() {
    let out_dir = env::var("OUT_DIR").unwrap();
    let memory_x_path = Path::new(&out_dir).join("memory.x");

    // 根据目标平台生成不同的内存布局
    let target = env::var("TARGET").unwrap_or_default();
    let memory_layout = match target.as_str() {
        "thumbv7em-none-eabihf" => {
            // STM32F4系列
            r#"
MEMORY
{
  /* NOTE 1 K = 1 KiBi = 1024 bytes */
  /* STM32F401RE: 512K flash, 96K RAM */
  FLASH : ORIGIN = 0x08000000, LENGTH = 512K
  RAM : ORIGIN = 0x20000000, LENGTH = 96K
}

/* This is where the call stack will be allocated. */
/* The stack is of the full descending type. */
/* You may want to use this variable to locate the call stack and static
   variables in different memory regions. Below is shown the default value */
_stack_start = ORIGIN(RAM) + LENGTH(RAM);

/* You can use this symbol to customize the location of the .text section */
/* If omitted the .text section will be placed right after the .vector_table
   section */
/* This is required only on microcontrollers that store some configuration right
   after the vector table */
/* _stext = ORIGIN(FLASH) + 0x400; */

/* Example of putting non-initialized variables into custom RAM locations. */
/* This assumes you have defined a region RAM2 above, and in the Rust
   sources added the attribute `#[link_section = ".ram2bss"]` to the data
   you want to place there. */
/* Note that the section will not be zero-initialized by the runtime! */
/* SECTIONS {
     .ram2bss (NOLOAD) : ALIGN(4) {
       *(.ram2bss);
       . = ALIGN(4);
     } > RAM2
   } INSERT AFTER .bss;
*/
"#
        }
        "thumbv6m-none-eabi" => {
            // STM32F0系列
            r#"
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 256K
  RAM : ORIGIN = 0x20000000, LENGTH = 32K
}

_stack_start = ORIGIN(RAM) + LENGTH(RAM);
"#
        }
        _ => {
            // 默认配置
            r#"
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 256K
  RAM : ORIGIN = 0x20000000, LENGTH = 64K
}

_stack_start = ORIGIN(RAM) + LENGTH(RAM);
"#
        }
    };

    fs::write(&memory_x_path, memory_layout).expect("Failed to write memory.x");
    println!("cargo:rustc-link-search={}", out_dir);
}

fn generate_linker_script() {
    let out_dir = env::var("OUT_DIR").unwrap();
    let link_x_path = Path::new(&out_dir).join("link.x");

    let linker_script = r#"
/* Linker script for embedded applications */

INCLUDE memory.x

/* # Entry point = reset vector */
EXTERN(Reset);
ENTRY(Reset);

/* # Exception vectors */
EXTERN(__EXCEPTIONS);

/* # Pre-initialization function */
/* If the user overrides this using the `#[pre_init]` attribute or by defining a
   `__pre_init` function, then the function this points to will be called before
   the RAM is initialized. */
PROVIDE(__pre_init = DefaultPreInit);

/* # Sections */
SECTIONS
{
  PROVIDE(_stack_start = ORIGIN(RAM) + LENGTH(RAM));

  /* ## Sections in FLASH */
  /* ### Vector table */
  .vector_table ORIGIN(FLASH) :
  {
    /* Initial Stack Pointer (SP) value */
    LONG(_stack_start);

    /* Reset vector */
    KEEP(*(.vector_table.reset_vector));

    /* Exceptions */
    KEEP(*(.vector_table.exceptions));
  } > FLASH

  PROVIDE(_stext = ADDR(.vector_table) + SIZEOF(.vector_table));

  /* ### .text */
  .text _stext :
  {
    *(.Reset);

    *(.text .text.*);
    *(.HardFaultTrampoline);
    *(.HardFault.*);

    /* The HardFaultTrampoline uses the `b` instruction to enter `HardFault`,
       so must be placed close to it. */
    . = ALIGN(4);
    __etext = .;
  } > FLASH

  /* ### .rodata */
  .rodata : ALIGN(4)
  {
    *(.rodata .rodata.*);

    /* 4-byte align the end (VMA) of this section.
       This is required by LLD to ensure the LMA of the following .data
       section will have the correct alignment. */
    . = ALIGN(4);
  } > FLASH

  /* ## Sections in RAM */
  /* ### .data */
  .data : ALIGN(4)
  {
    . = ALIGN(4);
    __sdata = .;
    *(.data .data.*);
    . = ALIGN(4); /* 4-byte align the end (VMA) of this section */
  } > RAM AT > FLASH

  /* Allow sections from user `memory.x` injected using `INSERT AFTER .data` to
   * use the .data loading mechanism by pushing __edata. Note: do not change
   * output region or load region in those user sections! */
  . = ALIGN(4);
  __edata = .;

  /* LMA of .data */
  __sidata = LOADADDR(.data);

  /* ### .gnu.sgstubs */
  /* This section contains the TrustZone-M veneers put there by the Arm GNU linker. */
  /* This memory region should only be used on TrustZone-M capable targets. */
  .gnu.sgstubs : ALIGN(32)
  {
    . = ALIGN(32);
    __veneer_base = .;
    *(.gnu.sgstubs*)
    . = ALIGN(32);
    __veneer_limit = .;
  } > RAM

  /* ### .bss */
  .bss (NOLOAD) : ALIGN(4)
  {
    . = ALIGN(4);
    __sbss = .;
    *(.bss .bss.*);
    *(COMMON); /* Uninitialized C statics */
    . = ALIGN(4); /* 4-byte align the end (VMA) of this section */
  } > RAM

  /* Allow sections from user `memory.x` injected using `INSERT AFTER .bss` to
   * use the .bss zeroing mechanism by pushing __ebss. Note: do not change
   * output region in those user sections! */
  . = ALIGN(4);
  __ebss = .;

  /* ### .uninit */
  .uninit (NOLOAD) : ALIGN(4)
  {
    . = ALIGN(4);
    __suninit = .;
    *(.uninit .uninit.*);
    . = ALIGN(4);
    __euninit = .;
  } > RAM

  /* Place the heap right after `.uninit` in RAM */
  PROVIDE(__sheap = __euninit);

  /* ## .got */
  /* Dynamic relocations are unsupported. This section is only used to detect relocatable code in
     the input files and raise an error if relocatable code is found */
  .got (NOLOAD) :
  {
    KEEP(*(.got .got.*));
  }

  /* ## Discarded sections */
  /DISCARD/ :
  {
    /* Unused exception related info that only wastes space */
    *(.ARM.exidx);
    *(.ARM.exidx.*);
    *(.ARM.extab.*);
  }
}

/* Do not exceed this mark in the error messages below                                    | */
/* # Alignment checks */
ASSERT(ORIGIN(FLASH) % 4 == 0, "
ERROR(cortex-m-rt): the start of the FLASH region must be 4-byte aligned");

ASSERT(ORIGIN(RAM) % 4 == 0, "
ERROR(cortex-m-rt): the start of the RAM region must be 4-byte aligned");

ASSERT(__sdata % 4 == 0 && __edata % 4 == 0, "
BUG(cortex-m-rt): .data is not 4-byte aligned");

ASSERT(__sidata % 4 == 0, "
BUG(cortex-m-rt): the LMA of .data is not 4-byte aligned");

ASSERT(__sbss % 4 == 0 && __ebss % 4 == 0, "
BUG(cortex-m-rt): .bss is not 4-byte aligned");

ASSERT(__sheap % 4 == 0, "
BUG(cortex-m-rt): start of .heap is not 4-byte aligned");

/* # Position checks */

/* ## .vector_table */
ASSERT(__reset_vector == ADDR(.vector_table) + 0x4, "
BUG(cortex-m-rt): the reset vector is missing");

ASSERT(__eexceptions == ADDR(.vector_table) + 0x40, "
BUG(cortex-m-rt): the exception vectors are missing");

ASSERT(SIZEOF(.vector_table) > 0x40, "
ERROR(cortex-m-rt): The interrupt vectors are missing.
Possible solutions, from most likely to less likely:
- Link to a device crate
- Disable the 'device' default feature to build a generic application (a dependency
may be enabling it)
- Supply the interrupt handlers yourself. Check the documentation for details.");

/* ## .text */
ASSERT(ADDR(.vector_table) + SIZEOF(.vector_table) <= _stext, "
ERROR(cortex-m-rt): The .text section can't be placed inside the .vector_table section
Set _stext to an address greater than 'ORIGIN(FLASH) + 0x40'");

ASSERT(_stext + SIZEOF(.text) < ORIGIN(FLASH) + LENGTH(FLASH), "
ERROR(cortex-m-rt): The .text section must be placed inside the FLASH memory.
Set _stext to a smaller value or increase the size of the FLASH region.");

/* # Other checks */
ASSERT(SIZEOF(.got) == 0, "
ERROR(cortex-m-rt): .got section detected in the input object files
Dynamic relocations are not supported. If you are linking to C code compiled using
the 'cc' crate then modify your build script to compile the C code _without_
the -fPIC flag. See the documentation of the `cc::Build.pic` method for details.");
/* Do not exceed this mark in the error messages above                                    | */
"#;

    fs::write(&link_x_path, linker_script).expect("Failed to write link.x");
}

fn generate_version_info() {
    let version = env::var("CARGO_PKG_VERSION").unwrap_or_else(|_| "0.0.0".to_string());
    let name = env::var("CARGO_PKG_NAME").unwrap_or_else(|_| "unknown".to_string());
    let authors = env::var("CARGO_PKG_AUTHORS").unwrap_or_else(|_| "unknown".to_string());
    let description = env::var("CARGO_PKG_DESCRIPTION").unwrap_or_else(|_| "".to_string());

    println!("cargo:rustc-env=PKG_VERSION={}", version);
    println!("cargo:rustc-env=PKG_NAME={}", name);
    println!("cargo:rustc-env=PKG_AUTHORS={}", authors);
    println!("cargo:rustc-env=PKG_DESCRIPTION={}", description);

    // 生成版本信息头文件
    let out_dir = env::var("OUT_DIR").unwrap();
    let version_h_path = Path::new(&out_dir).join("version.h");

    let version_header = format!(
        r#"
#ifndef VERSION_H
#define VERSION_H

#define VERSION_MAJOR {}
#define VERSION_MINOR {}
#define VERSION_PATCH {}
#define VERSION_STRING "{}"
#define BUILD_DATE "{}"
#define GIT_HASH "{}"

#endif // VERSION_H
"#,
        version.split('.').nth(0).unwrap_or("0"),
        version.split('.').nth(1).unwrap_or("0"),
        version.split('.').nth(2).unwrap_or("0"),
        version,
        chrono::Utc::now().format("%Y-%m-%d %H:%M:%S UTC"),
        get_git_hash().unwrap_or_else(|| "unknown".to_string())
    );

    fs::write(&version_h_path, version_header).expect("Failed to write version.h");
}

fn configure_build_dependencies() {
    // 配置构建时依赖
    let target = env::var("TARGET").unwrap_or_default();
    
    // 根据目标平台配置不同的链接选项
    match target.as_str() {
        "thumbv7em-none-eabihf" => {
            println!("cargo:rustc-link-arg=-Tlink.x");
            println!("cargo:rustc-link-arg=--nmagic");
        }
        "thumbv6m-none-eabi" => {
            println!("cargo:rustc-link-arg=-Tlink.x");
            println!("cargo:rustc-link-arg=--nmagic");
        }
        _ => {
            // 其他目标平台的配置
        }
    }

    // 配置优化选项
    let profile = env::var("PROFILE").unwrap_or_default();
    match profile.as_str() {
        "release" => {
            println!("cargo:rustc-link-arg=-Os"); // 优化大小
        }
        "debug" => {
            println!("cargo:rustc-link-arg=-Og"); // 优化调试体验
        }
        _ => {}
    }
}