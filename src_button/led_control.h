/*
    Header file (interface - specification file) for the leds
*/
#include <stdint.h>

/*
    Include Guard - see led.h for more details
*/
#ifndef LED_CONTROL_H_
#define LED_CONTROL_H_

/*
    Define LED locations - found in user manual
*/
#define LED_GREEN 12
#define LED_ORANGE 13
#define LED_RED 14
#define LED_BLUE 15

/*
    Prototypes
*/
void init_leds(void);
void led_on(uint8_t led_no);
void led_off(uint8_t led_no);

#endif //LED_H_