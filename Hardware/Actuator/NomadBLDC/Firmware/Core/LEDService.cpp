
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

// Primary Include
#include "LEDService.h"

// C System Files

// C++ System Files

// Project Includes
#include "mbed.h"

LEDService::LEDService() : led_pin_(nullptr), initialized_(false)
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

}

void LEDService::On()
{
    if(initialized_)
        led_pin_->write(1);
}

void LEDService::Off()
{
    if(initialized_)
        led_pin_->write(0);
}

void LEDService::Blink(blink_pattern_t blink_pattern)
{
    // TODO: Send Blink Signal to Service
}

void LEDService::Blink(uint32_t on_period, uint32_t off_period)
{
    // TODO: Send Blink Signal to Service
}

void LEDService::Init(PinName led_pin)
{
    led_pin_ = new DigitalOut(led_pin);
    initialized_ = true;
}

LEDService &LEDService::Instance()
{
    static LEDService instance;
    return instance;
}