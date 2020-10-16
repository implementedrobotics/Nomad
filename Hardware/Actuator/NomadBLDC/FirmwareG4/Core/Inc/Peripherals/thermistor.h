/*
 * thermistor.h
 *
 *  Created on: October 15, 2020
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

#ifndef CORE_PERIPHERAL_THERMISTOR_H_
#define CORE_PERIPHERAL_THERMISTOR_H_

// C System Files
#include <stdlib.h>
#include <stdint.h>

// C++ System Files

// STM32 System Files
#include <main.h> 

// Project Includes


class Thermistor
{

public:
    static constexpr float kADCMaxValue = 4096.0f;

    // Constructor
    Thermistor(ADC_TypeDef *ADC, float Beta, float R_0, float R_bal, size_t lut_size = 64);

    // Sample Temperature from Thermistor
    float SampleTemperature();

    // Compute Thermistor Temperature from ADC Value
    float ComputeTempValue(uint16_t counts);

    // Generate Thermistor Table
    void GenerateTable();

private:

    // STM32 ADC Device Pointer
    ADC_TypeDef *ADC_;

    // Thermistor Beta Value
    float Beta_;

    // Thermistor Nominal Resistance Value
    float R_0_;

    // Thermistor Balance Resistance Value
    float R_bal_;

    // Thermistor Lookup Table Size
    size_t lut_size_;

    // Thermistor Lookup Table
    float *temp_lut_;

};


#endif // CORE_PERIPHERAL_THERMISTOR_H_
