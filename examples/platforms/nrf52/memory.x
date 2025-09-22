/* nRF52840 内存布局 */
/* 可根据具体nRF52型号调整 */

MEMORY
{
  /* Flash存储器 - 1MB */
  /* 为SoftDevice预留空间 */
  FLASH : ORIGIN = 0x00027000, LENGTH = 868K
  
  /* SRAM - 256KB */
  /* 为SoftDevice预留空间 */
  RAM : ORIGIN = 0x20020000, LENGTH = 128K
}

/* 如果不使用SoftDevice，可以使用完整内存 */
/*
MEMORY
{
  FLASH : ORIGIN = 0x00000000, LENGTH = 1024K
  RAM : ORIGIN = 0x20000000, LENGTH = 256K
}
*/

/* 栈顶位置 */
_stack_start = ORIGIN(RAM) + LENGTH(RAM);

/* 堆大小配置 */
_heap_size = 32K;

/* 中断向量表位置 */
PROVIDE(_stext = ORIGIN(FLASH));

/* SoftDevice配置 */
/* S140 SoftDevice占用空间 */
_softdevice_flash_start = 0x00000000;
_softdevice_flash_size = 0x27000;  /* 156KB */
_softdevice_ram_start = 0x20000000;
_softdevice_ram_size = 0x20000;    /* 128KB */

/* 应用程序区域 */
_app_flash_start = ORIGIN(FLASH);
_app_ram_start = ORIGIN(RAM);