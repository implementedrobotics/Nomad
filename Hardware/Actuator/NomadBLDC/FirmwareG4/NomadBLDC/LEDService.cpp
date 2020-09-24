
/*
 * LEDService.cpp
 *
 *  Created on: August 26, 2019
 *      Author: Quincy Jones
 *
 * Copyright (c) <2019> <Quincy Jones - quincy@implementedrobotics.com/>
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software
 * is furnished to do so, subject to the following conditions
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

// Primary Include
#include <LEDService.h>

// C System Files

// C++ System Files

// Project Includes
#include "cmsis_os2.h"

osMessageQueueId_t led_queue = 0;
osThreadId_t led_thread = 0;

LEDService::LEDService() : port_(nullptr), pin_mask_(0), initialized_(false), thread_active_(false)
{
    blink_patterns_[SLOW].on_time = 1000;
    blink_patterns_[SLOW].off_time = 1000;

    blink_patterns_[MEDIUM].on_time = 500;
    blink_patterns_[MEDIUM].off_time = 500;

    blink_patterns_[FAST].on_time = 100;
    blink_patterns_[FAST].off_time = 100;

    blink_patterns_[PATTERN_1].on_time = 500;
    blink_patterns_[PATTERN_1].off_time = 100;

    blink_patterns_[PATTERN_2].on_time = 100;
    blink_patterns_[PATTERN_2].off_time = 500;

    blink_patterns_[ON].on_time = 500;
    blink_patterns_[ON].off_time = 0;

    blink_patterns_[OFF].on_time = 0;
    blink_patterns_[OFF].off_time = 500;
}

void LEDService::On()
{
    if (initialized_) {
        if(thread_active_) 
        {
            osThreadSuspend(led_thread);
            thread_active_ = false;
        }
        LL_GPIO_SetOutputPin(port_, pin_mask_);
    }
}

void LEDService::Off()
{
    if (initialized_)
    {
        if(thread_active_) 
        {
            osThreadSuspend(led_thread);
            thread_active_ = false;
        }

        LL_GPIO_ResetOutputPin(port_, pin_mask_);
    }
}

void LEDService::Toggle()
{
    if (initialized_)
    {
        if(thread_active_) 
        {
            osThreadSuspend(led_thread);
            thread_active_ = false;
        }

        LL_GPIO_TogglePin(port_, pin_mask_);
    }
}

void LEDService::Blink(blink_pattern_t blink_pattern)
{
    if (initialized_)
    {
        if (!thread_active_)
        {
            osThreadResume(led_thread);
            thread_active_ = true;
        }

        osMessageQueuePut(led_queue, &blink_pattern, 0, 0); // Send Data to Queue and Leave w/o timeout
    }
}

void LEDService::Blink(uint32_t on_period, uint32_t off_period)
{
    if (initialized_)
    {
        if (!thread_active_)
        {
            osThreadResume(led_thread);
            thread_active_ = true;
        }

        LEDService::blink_timing_t pattern;
        pattern.on_time = on_period;
        pattern.off_time = off_period;
        osMessageQueuePut(led_queue, &pattern, 0, 0); // Send Data to Queue and Leave w/o timeout
    }
}

void LEDService::Init(GPIO_TypeDef *GPIOx, uint32_t PinMask)
{
    port_ = GPIOx;
    pin_mask_ = PinMask;

    LL_GPIO_ResetOutputPin(port_, pin_mask_); // Start in off condition
    initialized_ = true;
    thread_active_ = false;
    osThreadSuspend(led_thread);
}

LEDService &LEDService::Instance()
{
    static LEDService instance;
    return instance;
}

extern "C" void status_led_thread(void *arg)
{
    // TODO: Suspend Thread when not in a flash mode
    led_queue = osMessageQueueNew(10, sizeof(LEDService::blink_timing_t), NULL);
    led_thread = osThreadGetId();

    //LEDService &led_ = LEDService::Instance();
    LEDService::Instance().Init(LED_STATUS_GPIO_Port, LED_STATUS_Pin);

    // Start in off mode
    LEDService::blink_timing_t pattern;
    pattern.off_time = 500;
    pattern.on_time = 0;
    //osThreadSuspend(NULL);

    for (;;)
    {
        osMessageQueueGet(led_queue, &pattern, 0, 0);
        LL_GPIO_SetOutputPin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
        osDelay(pattern.on_time);
        LL_GPIO_ResetOutputPin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
        osDelay(pattern.off_time);
    }

    osThreadExit();
}
