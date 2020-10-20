/*
 * adc.h
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

#ifndef CORE_PERIPHERAL_ADC_H_
#define CORE_PERIPHERAL_ADC_H_

// C System Files

// C++ System Files

// Project Includes
#include <Utilities/lpf.h>
#include <main.h>

// TODO:
// TYPE: Polling/Triggered/Interrupt
// Bias
// Filter
// Read Value
// ADC Interrupt Callback
// Register Callback Function

class ADCDevice
{

public:

    // TODO: Triggered vs Polled. 
    // TODO: DMA Eventually

    ADCDevice(ADC_TypeDef *ADC);

    // Enable ADC
    void Enable();

    // Start Sampling
    void Start();

    // Stop Sampling
    void Stop();

    // Enable Interrupt
    void EnableInterrupt();

    // Read ADC Value
    uint16_t Read();

private:

    ADC_TypeDef *ADC_;
    
    // ADC Bias
    int16_t bias_;

    // Current ADC Sample Valuer
    int16_t value_;

    // Low Pass Filter
    LowPassFilter filter_;

    // Low Pass Filter Enabled
    bool enable_filter_;

};

#endif // CORE_PERIPHERAL_ADC_H_