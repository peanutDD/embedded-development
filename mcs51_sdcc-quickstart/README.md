# 89C52 例程

89C52 的点灯例程，使用 SDCC 编译器进行编译

```c
#include <reg52.h>
#include <stdio.h>

void delay_100ms(void) //@12MHz
{
    unsigned char i, j;

    i = 195;
    j = 138;

    do
    {
        while (--j)
            ;
    } while (--i);
}

void main(void)
{
    while (1)
    {
        P00 = !P00;
        delay_100ms();
    }
}
```

## SDCC 编译设置

默认设置：`--iram-size 256 --xram-size 0 --code-size 8192`，对应参数如下：

- IRAM  SIZE: 256  Bytes
- XRAM  SIZE: 0    Bytes
- FLASH SIZE: 8192 Bytes

要修改上述参数，请到 `构建器选项 -> Other Global Options` 中进行修改。
