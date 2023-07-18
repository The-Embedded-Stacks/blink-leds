/*
    Target Device: STM32F407VGT (Discovery Board)
    Basic bare-metal program to blink the on-board LEDs (PD12 -PD15) at different rates
    The blinking LEDs should appear to flashing concurrently - however concurrencey is not used
*/
#include <stdint.h>
#include "main.h"
#include "led_control.h"

/*
    Input: N/A
    Output: Blink the LEDs at different rates
    Function: Forever loop that blinks the on-board LEDs in order
*/
int main(void)
{
    uint32_t master_time = 0;

    init_leds(); // Enable clocks - Set modes - Turn OFF LEDs

    /*
        Main loop
        master_time and remainder control when a LEDs state is toggled

    */
    while(1)
    {
        master_time++;

        if(master_time % DELAY_1S == 0) // SLOW RATE
            toggle_led(LED_GREEN);
        if(master_time % DELAY_500MS == 0) // MEDIUM RATE
            toggle_led(LED_ORANGE);        
        if(master_time % DELAY_250MS == 0) // FAST RATE
            toggle_led(LED_RED);
        if(master_time % DELAY_125MS == 0) // VERY FAST RATE
            toggle_led(LED_BLUE);
    }

    return 0; // Will never be reached - precaution

}

/*
    Input: LED # (12-15)
    Output: LED ON or OFF
    Function: Fetch the current state of the passed LED and flip the state
*/
void toggle_led(uint8_t led)
{
    if (get_led_status(led))
    {
        led_off(led);
    }
    else
    {
        led_on(led);
    }
}
