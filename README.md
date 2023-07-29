# blink-leds
## There are two bare-metal programs for the STM32F407 Discovery Board within this repo:
1. Simple program to blink the on-borad LEDs (PD12 -PD15) at different rates found in [src_simple](src_simple/main.c).
    - __NOTE:__ Concurrency is not explored in this program
2. A program that incorporates the user button (PA0) and reset button (nRST) to control the on-borad LEDs found in [src_button](src_button/main.c).


# Simple LED Program Breakdown
- led_control
    - initialize registers
    - function definitions for led ON, led OFF, led STATUS
- main
    - super loop to toogle the on-board LEDs in succession
- Startup File 
    - initialize exceptions and interrupt handelers
    - only the Reset Handler was defined
- Linker Script
- Makefile
- System Calls
    - syscalls exist because I was getting compalation errors and didn't have the time to track down where they were entering the system

# Button Controlled LED Program Breakdown
- hardware_init
- button_control
    - configuration
- led_control
    - configuration
    - function definitions for led ON, led OFF, led STATUS
- main
    - super loop to toogle the on-board LEDs in succession - controlled with the user button (PA0)
- Startup File
    - initialize exceptions and interrupt handelers
    - defined: Reset Handler, EXTI0 IRQ, & TIM2 IRQ 
- Linker Script
- Makefile
- System Calls