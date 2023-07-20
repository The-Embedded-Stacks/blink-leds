/*
    HEADER
*/
#include <stdint.h>
#include "hardware_init.h"

/*

*/
void init_clocks(void)
{
    /* 
        Enable clock for peripheral - the AHB1 RCC base address is 0x40023800
        Go to the reference manual RCC section and find the enable offset 0x30
        0x4002 3800 + 0x30 = 0x40023830 
    */
    uint32_t* pRCC_ANB1ENR = (uint32_t*)(RCC_BASE + 0x30);

    /*
        Bit 3 GPIODEN: IO port D clock enable - found in reference manual
        Dereference & perform bitwise operation to enable
    */
    *pRCC_ANB1ENR |= (1 << 3);

}