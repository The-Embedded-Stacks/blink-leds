/*
    This file initializes the neseccary registers to enables the on-board LEDs (12-15)
    It also contains the function definitions to toggle the individual LEDs
*/
#include <stdint.h>
#include "led_control.h"


void init_leds(void)
{
    /* 
        Enable clock for peripheral - the AHB1 RCC base address is 0x40023800
        Go to the reference manual RCC section and find the enable offset 0x30
        0x4002 3800 + 0x30 = 0x40023830 
    */
    uint32_t* pRCC_ANB1ENR = (uint32_t*)0x40023830;

    /* 
        Need to access correct port mode register to configure IO direction mode
            * 00: Input (reset state)
            * 01: General purpose output mode
            * 10: Alternate function mode 
            * 11: Analog mode
        We are accessing on-board LEDs PD12 to PD15 (PD = Port-D) - found in user manual
        Cross-ref with datasheet to find GPIOD base address under parent peripheral AHB1
            * There is no offset
    */
    uint32_t* pGPIOD_MODER = (uint32_t*)0x40020C00;

    /*
        Will need to access the output data register for GPIOD
        This 32-bit register is dedicated for the 16 usable pins (0-15) - (16-31) reserved
        uint32_t* pGPIOD_ODR = (uint32_t*)0x40020C14;
    */

    /*
        Bit 3 GPIODEN: IO port D clock enable - found in reference manual
        Dereference & perform bitwise operation to enable
    */
    *pRCC_ANB1ENR |= (1 << 3);

    /*
        Configure the pins to 01: General purpose output mode
        Bits 2y:2y+1 MODERy[1:0]: Port x configuration bits (y = 0..15)
            * Example pin 12 (green LED)
            * 2(12) = bit 24: set to 1 (general output mode)
            * 2(12) + 1 = bit 25: set to 0 (gneral output mode)
        NOTE: setting the odd bit to zero isn't necessarily required since is defaulted to 0
    */
    *pGPIOD_MODER |= (1U << (2 * LED_GREEN)); //set 24 to 1
    *pGPIOD_MODER &= ~(1U << ((2 * LED_GREEN) + 1)); //set 25 to 0

    //y=13 - orange
    *pGPIOD_MODER |= (1U << (2 * LED_ORANGE)); //set 26 to 1
    *pGPIOD_MODER &= ~(1U << ((2 * LED_ORANGE) + 1)); //set 27 to 0

    //y=14 - red
    *pGPIOD_MODER |= (1U << (2 * LED_RED)); //set 28 to 1
    *pGPIOD_MODER &= ~(1U << ((2 * LED_RED) + 1)); //set 29 to 0
    
    //y=15 - blue
    *pGPIOD_MODER |= (1U << (2 * LED_BLUE)); //set 30 to 1
    *pGPIOD_MODER &= ~(1U << ((2 * LED_BLUE) + 1)); //set 31 to 0

    //Initially set the leds to OFF
    led_off(LED_GREEN);
    led_off(LED_ORANGE);
    led_off(LED_RED);
    led_off(LED_BLUE);
}

/*
    Intake: led # 12,13,14,15
    Output: led # n ON
    Function: Take in led # - bitwise shift (OR) to set that pin HIGH
*/
void led_on(uint8_t led_num)
{
    /*
        Need to access the data register for GPIOD
        Since we will use the same address in both function calls and it doesn't change we could 
        declare the address as a global constant - #define GPIOD_ODR_ADDRESS (uint32_t*)0x40020C14
        This would avoid having to destroy and create the pointer every time a function is called
        However the global pointer will take up 4 bytes of static memory 
        therefore memory consumption is essentially a wash - performance hit is negliable as well
    */
    uint32_t* pGPIOD_ODR = (uint32_t*)0x40020C14;
    *pGPIOD_ODR |= (1U << led_num);
}

/*
    Intake: led # 12,13,14,15
    Output: led # n OFF
    Function: Take in led # - bitwise shift (AND) to set that pin LOW
*/
void led_off(uint8_t led_num)
{
    uint32_t* pGPIOD_ODR = (uint32_t*)0x40020C14; // access the data register for GPIOD
    *pGPIOD_ODR &= ~(1U << led_num);
}

/*
    Intake: led # 12,13,14,15
    Output: LED Staus - 0 or 1 (OFF or ON)
    Function: Take in led # and check where the position is set to 1 or 0
*/
uint8_t get_led_status(uint8_t led_num)
{
    uint32_t status = 0;
    uint32_t* pGPIOD_ODR = (uint32_t*)0x40020C14;

    status = *pGPIOD_ODR & (1U << led_num);

    return (status != 0); // return true or false
}