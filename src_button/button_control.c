/*
    HEADER
*/
#include <stdint.h>
#include "button_control.h"

/*
    Initialize Registers
*/
void init_user_button(void)
{
    /*
        The STM32F4 Discovery Board has a on-board user push button (blue)
        From the user manual we see that this button is connected to PA0 (Port A - Pin 0)
        From the datasheet (Table 10) we see that PA0 is in the AHB1 bus with a base address of 0x4002 0000
        From the reference manual we need the following configurations knowing PA0 is default LOW:
            * MODER - address offest = 0x00 - set to 00 (input)
            * OTYPER - N/A
            * OSPEEDR - N/A
            * PUPDR - address offest = 0x0C - set to 10 (pull down)
            * IDR - address offset = 0x10 - set to an input so we will read from this register (bit 0)
            * ODR - N/A
            * BSRR - N/A
            * LCKR - N/A
            * AFRL - N/A
            * AFRH - N/A
    */
   uint32_t* pGPIOA_MODER = (uint32_t*)(GPIOA_BASE + 0x00);
   uint32_t* pGPIOA_PUPDR = (uint32_t*)(GPIOA_BASE + 0x0C);

    /*
        Configure the pin (PA0) to 00: Input
        Bits 2y:2y+1 MODERy[1:0]: Port x configuration bits (y = 0..15)
        NOTE: the pins are 00 by default so performing this is redundant
    */
   *pGPIOA_MODER &= ~(1U << 0); //set bit 0 LOW
   *pGPIOA_MODER &= ~(1U << 1); //set bit 1 LOW

    /*
        Configure the pin (PA0) to 10: Pull-down
        Bits 2y:2y+1 MODERy[1:0]: Port x configuration bits (y = 0..15)
    */
   *pGPIOA_PUPDR &= ~(1U << 0); //set bit 0 LOW
   *pGPIOA_PUPDR |= (1U << 1); //set bit 1 HIGH

}