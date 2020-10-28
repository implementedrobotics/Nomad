/*
 * nomad_hw.h
 *
 *  Created on: September 24, 2020
 *      Author: Quincy Jones
 *
 * Copyright (c) <2020> <Quincy Jones - quincy@implementedrobotics.com/>
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

#ifndef NOMAD_HW_H_
#define NOMAD_HW_H_

#ifdef __cplusplus
extern "C" {
#endif

// HAL Timer for Tick Reference
#define TICK_TIMER TIM6

#define VERSION_MAJOR 2
#define VERSION_MINOR 0


// FET Thermistor Parameters
#define FET_THERM_BETA 3455.0f // 
#define FET_THERM_RESISTANCE 10000.0f // 10k
#define FET_THERM_RESISTANCE_BAL 10000.0f // 10k
#define FET_THERM_LUT_SIZE 64

// TODO: Should have a better macro for handling different optimization types
#if defined(USE_CCM)
#define CCM_ATTRIBUTE __attribute__ ((section (".ccmram")))
#else
#define CCM_ATTRIBUTE // Blank
#endif

#ifdef __cplusplus
}
#endif

#endif // NOMAD_HW_H_