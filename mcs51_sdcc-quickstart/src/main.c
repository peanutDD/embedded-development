/**
 * CPU: 89C52
 * Freq: 12MHz
*/

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
