/*
 * thermistor.cpp
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

// Primary Include
#include <Peripherals/thermistor.h>

// C++ Includes
#include <cmath>
#include <algorithm>

// C System Files
#include <stdint.h>
#include <stdlib.h>

// Project Includes
#include <main.h> // STM32 Driver Includes

Thermistor::Thermistor(ADC_TypeDef *ADC, float Beta, float R_0, float R_bal, size_t lut_size ) : ADC_(ADC), Beta_(Beta), R_0_(R_0), R_bal_(R_bal), lut_size_(lut_size), temp_lut_(nullptr)
{
    EnableADC(ADC);
}

float Thermistor::SampleTemperature()
{
    // Read ADC
    uint16_t counts = PollADC(ADC_);

    // Have LUT?  If not compute and return
    if(temp_lut_ == nullptr)
        return ComputeTempValue(counts);

    // Compute LUT Index and Round Down
    size_t idx_1 =  static_cast<size_t>(counts * lut_size_ / kADCMaxValue);
    
    // Clamp to Max
    idx_1 = std::min(idx_1, lut_size_ - 2);

    // Lookup Temps to LERP    
    float temp_1 = temp_lut_[idx_1];
    float temp_2 = temp_lut_[idx_1+1];

    // Compute LERP t value
    float t = (counts - idx_1 * lut_size_) / static_cast<float>(lut_size_); 

    // Compute Temp LERP value for sample
    float temp_sample = temp_1 + t * (temp_2 - temp_1);

    return temp_sample;
}

float Thermistor::ComputeTempValue(uint16_t counts)
{
    // Calc FET Temp
    float V_out = 3.3f * counts / kADCMaxValue;
    float R_th = 3.3f * R_bal_ / V_out - R_bal_;

    // https://www.digikey.com/en/maker/projects/how-to-measure-temperature-with-an-ntc-thermistor/4a4b326095f144029df7f2eca589ca54
    return 1.0f / (1.0f / (273.15f + 25.0f) + (1.0f / Beta_) * log(R_th / R_0_)) - 273.15f;
}
void Thermistor::GenerateTable()
{
    // Delete old LUT
    if(temp_lut_ == nullptr)
        delete[] temp_lut_;

    // Create a new LUT
    temp_lut_ = new float[lut_size_];

    // Compute Step Size
    uint32_t step = kADCMaxValue / lut_size_;

    for(size_t i = 0; i < lut_size_; i++)
    {
        // Compute LUT Value
        temp_lut_[i] = ComputeTempValue(i*step);
    }
}