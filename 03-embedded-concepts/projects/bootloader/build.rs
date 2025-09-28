use std::env;
use std::fs;
use std::path::Path;

fn main() {
  let out_dir = env::var("OUT_DIR").unwrap();
  let target = env::var("TARGET").unwrap();

  // 根据构建目标选择内存布局文件
  if env::var("CARGO_BIN_NAME").unwrap_or_default() == "bootloader" {
    // Bootloader 构建
    println!("cargo:rustc-link-search={}", out_dir);
    println!("cargo:rustc-link-arg=-Tmemory-bootloader.x");

    // 复制 bootloader 内存布局文件
    let memory_x = include_str!("memory-bootloader.x");
    let out_path = Path::new(&out_dir).join("memory.x");
    fs::write(out_path, memory_x).unwrap();
  } else {
    // Application 构建
    println!("cargo:rustc-link-search={}", out_dir);
    println!("cargo:rustc-link-arg=-Tmemory-app.x");

    // 复制应用程序内存布局文件
    let memory_x = include_str!("memory-app.x");
    let out_path = Path::new(&out_dir).join("memory.x");
    fs::write(out_path, memory_x).unwrap();
  }

  // 告诉 Cargo 在这些文件改变时重新运行构建脚本
  println!("cargo:rerun-if-changed=memory-bootloader.x");
  println!("cargo:rerun-if-changed=memory-app.x");
  println!("cargo:rerun-if-changed=build.rs");

  // 设置链接器参数
  if target.starts_with("thumbv") {
    println!("cargo:rustc-link-arg=-Tlink.x");
    println!("cargo:rustc-link-arg=--nmagic");
  }
}
