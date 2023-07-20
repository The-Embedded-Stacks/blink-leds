/*
    Target Device: STM32F407VGT (Discovery Board)
    Function:
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
    uint32_t master_time = 0;
    init_hardware();
    init_leds(); // Enable clocks - Set modes - Turn OFF LEDs

    /*
        Main loop
    */
    while(1)
    {
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
