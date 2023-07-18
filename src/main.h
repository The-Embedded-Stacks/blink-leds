#include <stdint.h>

/*
    Include Guard - this is an older technique 
        * (in other languages we now use #pragma once but it can't be used in C or C++)
        * this technique safe guards from the header file being processed by the compiler 
          more than once (if it is included in multiple files)
        * If this wasn't done and the led.h was included in several files the compiler 
          would process it for every call and would cause to errors
*/
#ifndef MAIN_H_
#define MAIN_H_

/*
    Prototypes
*/
void toggle_led(uint8_t led);

#endif