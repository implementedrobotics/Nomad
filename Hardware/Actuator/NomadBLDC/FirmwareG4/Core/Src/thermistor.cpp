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

// C System Files
#include <stdint.h>
#include <stdlib.h>

// Project Includes
#include <main.h> // STM32 Driver Includes


Thermistor::Thermistor(ADC_TypeDef *ADC, float Beta, float R_0, float R_bal, size_t lut_size ) : ADC_(ADC), Beta_(Beta), R_0_(R_0), R_bal_(R_bal), lut_size_(lut_size), temp_lut_(nullptr)
{
}

float Thermistor::SampleTemperature()
{
    if(temp_lut_ == nullptr)
        return 5000; // Some unrelistic error value?

    // Read ADC
    uint16_t counts = PollADC(ADC_);

    // Compute LUT Index
    uint16_t lut_idx = counts * lut_size_ / static_cast<uint16_t>(kADCMaxValue);
    uint16_t lower_counts = lut_idx * kADCMaxValue / lut_size_;
    uint16_t upper_counts = (lut_idx+1) * kADCMaxValue / lut_size_;
    float t_0 = temp_lut_[lut_idx];
    float t_1 = temp_lut_[lut_idx+1];
    float t = static_cast<float>(counts - lower_counts) / static_cast<float>(upper_counts-lower_counts);
    float temp_sample = t_0 + t*(t_1-t_0);

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
    uint32_t step = kADCMaxValue / lut_size_;
    for(int i = 0; i < lut_size_; i++)
    {
        // Compute LUT Value
        temp_lut_[i] = ComputeTempValue(i*step);
    }
}