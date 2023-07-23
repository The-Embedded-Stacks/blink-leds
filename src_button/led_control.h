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
    Set up time delays (rates) for LEDs
    1ms is general used as the base in embedded applications which you can build other delays from
    You could do calculations to verify if 1250U acutally = 1ms 
    Same for the other delays - you could also monitor the circuits
*/
#define DELAY_1MS        1250U // this can be fine tuned
#define DELAY_1S  		(1000U * DELAY_1MS) // SLOW DELAY
#define DELAY_500MS     (500U  * DELAY_1MS) // MEDIUM DELAY
#define DELAY_250MS 	(250U  * DELAY_1MS) // FAST DELAY
#define DELAY_125MS 	(125U  * DELAY_1MS) // VERY FAST DELAY

/*
    Prototypes
*/
void init_leds(void);
void led_on(uint8_t led_no);
void led_off(uint8_t led_no);
uint8_t get_led_status(uint8_t led_num);

#endif //LED_H_