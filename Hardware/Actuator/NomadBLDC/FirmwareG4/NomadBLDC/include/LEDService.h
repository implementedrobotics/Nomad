/*
 * LED.h
 *
 *  Created on: August 27, 2019
 *      Author: Quincy Jones
 *
 * Copyright (c) <2019> <Quincy Jones - quincy@implementedrobotics.com/>
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software
 * is furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 * 
 */

#ifndef CORE_LED_H_
#define CORE_LED_H_

// C System Files

// C++ System Files

// Project Includes
#include "main.h"
#include <cmsis_os2.h>

class LEDService
{
public:

    typedef enum
    {
        SLOW = 0,
        MEDIUM = 1,
        FAST = 2,
        PATTERN_1 = 3,
        PATTERN_2 = 4,
        ON = 5,
        OFF = 6,
        NUM_PATTERNS
    } blink_pattern_t;

    struct blink_timing_t
    {
        uint32_t on_time;  // LED On Time (ms)
        uint32_t off_time; // LED Off Time (ms)
    };

    LEDService();
    void Init(GPIO_TypeDef *GPIOx, uint32_t PinMask);
    void On();
    void Off();
    void Blink(blink_pattern_t blink_pattern);
    void Blink(uint32_t on_period, uint32_t off_period); // Period in ms
    static LEDService &Instance();

private:

    GPIO_TypeDef *port_;
    uint32_t pin_mask_;

    uint32_t on_period_; // ms
    uint32_t off_period_; // ms

    bool initialized_; // Service Initialized

    blink_timing_t blink_patterns_[NUM_PATTERNS]; 
};

#ifdef __cplusplus
extern "C" {
#endif

void status_led_thread(void *arg);

#ifdef __cplusplus
}
#endif

#endif // CORE_LED_H_