/*
    Target Device: STM32F407VGT (Discovery Board)
    Function: This program utilizes the user button (PA0) and the on-board LEDs (PD12 - PD 15). From a reset state all LEDs will be OFF. When the user
    button is pressed the green LED (12) will turn ON followed by 13, 14, and 15 for every button press. Once 15 is ON the next button press will turn
    OFF the green LED followed by 13, 14, and 15 for every button press. This process is looped.
        * EXTI0_IRQ Handler is used to capture the user button input from PA0
        * TIM2_IRQ Handler is used as a debounce for the input signal - ensuring we only change the state of an LED every second if an input is recognized
*/
#include <stdint.h>
#include "hardware_init.h"
#include "led_control.h"
#include "button_control.h"
#include "main.h"

/*
    Input: N/A
    Output:
    Function:
*/
int main(void)
{
    init_hardware(); // Enable clocks
    init_user_button(); // Configure the user button (PA0)
    init_leds(); // Configure the LEDs - Turn OFF LEDs

    /*
        Main loop
    */
    while(1)
    {
        if (turn_off_flag == 0) {
            if (led_to_toggle == LED_GREEN) {
                led_on(LED_GREEN);
            }
            else if (led_to_toggle == LED_ORANGE) {
                led_on(LED_ORANGE);
            }
            else if (led_to_toggle == LED_RED) {
                led_on(LED_RED);
            }
            else if (led_to_toggle == LED_BLUE) {
                led_on(LED_BLUE);
            }
        } else {
            if (led_to_toggle == LED_GREEN) {
                led_off(LED_GREEN);
            }
            else if (led_to_toggle == LED_ORANGE) {
                led_off(LED_ORANGE);
            }
            else if (led_to_toggle == LED_RED) {
                led_off(LED_RED);
            }
            else if (led_to_toggle == LED_BLUE) {
                led_off(LED_BLUE);
            }
        } 
    }

}
