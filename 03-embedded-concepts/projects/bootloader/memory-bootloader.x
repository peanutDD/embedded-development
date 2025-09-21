/* 
 * Bootloader 内存布局
 * Bootloader 占用 Flash 前 32KB
 * Application 从 0x08008000 开始
 */

MEMORY
{
  /* Bootloader Flash - 32KB */
  FLASH : ORIGIN = 0x08000000, LENGTH = 32K
  
  /* Application Flash - 480KB */
  APP_FLASH : ORIGIN = 0x08008000, LENGTH = 480K
  
  /* SRAM - 96KB */
  RAM : ORIGIN = 0x20000000, LENGTH = 96K
}

/* 程序入口点 */
ENTRY(Reset);

/* 栈顶地址 */
_stack_start = ORIGIN(RAM) + LENGTH(RAM);

/* Application 起始地址 */
_app_start = ORIGIN(APP_FLASH);

SECTIONS
{
  /* 向量表 */
  .vector_table ORIGIN(FLASH) :
  {
    LONG(_stack_start);
    KEEP(*(.vector_table.reset_vector));
    KEEP(*(.vector_table.exceptions));
    KEEP(*(.vector_table.interrupts));
  } > FLASH

  /* Bootloader 代码段 */
  .text :
  {
    *(.text .text.*);
    . = ALIGN(4);
  } > FLASH

  /* 只读数据段 */
  .rodata :
  {
    *(.rodata .rodata.*);
    . = ALIGN(4);
  } > FLASH

  /* 初始化数据段 */
  .data : AT(ADDR(.rodata) + SIZEOF(.rodata))
  {
    _sdata = .;
    *(.data .data.*);
    . = ALIGN(4);
    _edata = .;
  } > RAM

  _sidata = LOADADDR(.data);

  /* 未初始化数据段 */
  .bss :
  {
    _sbss = .;
    *(.bss .bss.*);
    *(COMMON);
    . = ALIGN(4);
    _ebss = .;
  } > RAM

  /* 确保 bootloader 不超过 32KB */
  ASSERT(SIZEOF(.text) + SIZEOF(.rodata) + SIZEOF(.data) <= LENGTH(FLASH), 
         "Bootloader too large for allocated Flash space");
}