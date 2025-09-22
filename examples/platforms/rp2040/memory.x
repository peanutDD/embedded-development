/* RP2040 内存布局 */

MEMORY {
    /* Boot2引导程序 - 256字节 */
    BOOT2 : ORIGIN = 0x10000000, LENGTH = 0x100
    
    /* Flash存储器 - 2MB (减去Boot2) */
    FLASH : ORIGIN = 0x10000100, LENGTH = 2048K - 0x100
    
    /* SRAM - 264KB */
    /* 分为6个32KB的bank + 2个4KB的bank */
    RAM   : ORIGIN = 0x20000000, LENGTH = 264K
}

/* 外部Flash配置 */
EXTERN(BOOT2_FIRMWARE);

/* 栈顶位置 */
_stack_start = ORIGIN(RAM) + LENGTH(RAM);

/* 堆大小配置 */
_heap_size = 64K;

/* 多核配置 */
/* Core 1栈空间 */
_core1_stack_start = ORIGIN(RAM) + LENGTH(RAM) - 8K;

/* 共享内存区域 */
_shared_memory_start = ORIGIN(RAM) + LENGTH(RAM) - 16K;
_shared_memory_size = 8K;

/* PIO程序存储区域 */
_pio_program_start = ORIGIN(FLASH) + LENGTH(FLASH) - 4K;
_pio_program_size = 4K;

/* 中断向量表位置 */
PROVIDE(_stext = ORIGIN(FLASH));