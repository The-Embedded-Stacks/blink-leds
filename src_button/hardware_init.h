/*
    Header file (interface - specification file) for the hardware initializations
*/

/*
    Include Guard
*/
#ifndef HARDWARE_INIT_H_
#define HARDWARE_INIT_H_

/*
    Define base addresses
*/
// AHB1 Bus - Found in Reference Manual
#define RCC_BASE 0x40023800 // Reset and Clock Control
#define GPIOA_BASE 0x40020000
#define GPIOD_BASE 0x40020C00

// APB2 Bus - Found in Reference Manual
#define SYSCFG_BASE 0x40013800 // System Configuration Controller
#define EXTI_BASE 0x40013C00 // External Interrupt / Event Controller
#define EXTI0_IRQ 6 // EXTI0 - Position 6
#define TIM2_IRQ 28 // TIM2 - Position 28

// APB1 Bus - Found in Reference Manual
#define TIM2_BASE 0x40000000

// Nested Vectored Interupt Controller (NVIC) - Found in Arm® Cortex®-M4 Processor Technical Reference Manual
#define ISER0_BASE 0xE000E100 // Interrupt Set-Enable Register 0
#define IPR1_BASE 0xE000E400 // Interrupt Set-Enable Register 0

// LED control variables
extern volatile uint8_t led_to_toggle;
extern volatile uint8_t button_pressed_flag;
extern volatile uint8_t turn_off_flag;

/*
    Prototypes
*/
void init_hardware(void);

#endif