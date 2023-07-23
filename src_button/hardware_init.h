/*
    HEADER
*/

/*
    Include Guard
*/
#ifndef HARDWARE_INIT_H_
#define HARDWARE_INIT_H_

/*
    Define base addresses
*/
// AHB1
#define RCC_BASE 0x40023800 // Reset and Clock Control
#define GPIOA_BASE 0x40020000
#define GPIOD_BASE 0x40020C00

// APB2
#define SYSCFG_BASE 0x40013800 // System Configuration Controller
#define EXTI_BASE 0x40013C00 // External Interrupt / Event Controller
#define EXTI0_IRQ 6 // EXTI0 - Position 6

// Nested Vectored Interupt Controller (NVIC)
#define ISER0_BASE 0xE000E100 // Interrupt Set-Enable Register 0
#define IPR1_BASE 0xE000E400 // Interrupt Set-Enable Register 0

// LED control variable
volatile uint32_t led_to_toggle = 11;

/*
    Prototypes
*/


#endif