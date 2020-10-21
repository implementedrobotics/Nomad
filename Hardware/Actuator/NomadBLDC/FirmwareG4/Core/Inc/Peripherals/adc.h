/*
 * adc.h
 *
 *  Created on: October 20, 2020
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
#include <functional>

// STM32 System Files
#include <main.h>

// Project Includes
#include <Utilities/lpf.h>
#include <Logger.h>

class ADCDevice
{

public:

    // TODO: This should be up in a base "Peripheral Class"
    static constexpr int kMaxInterrupts = 200;

    // TODO: Triggered vs Polled. 
    // TODO: DMA Eventually, Continuous vs Single Converstion etc.
    // TODO: For now keep it simple.  Software, or Timer based at the moment
    // Assumes a pre "inited" ADC from CubeMX

    // Constructor
    ADCDevice(ADC_TypeDef *ADC);

    // Enable ADC
    void Enable();

    // Poll/Read a sample
    uint16_t Sample();

    // Start Sampling in Continuous/Timer Triggered Mode
    inline void Start(void) { LL_ADC_REG_StartConversion(ADC_); }

    // Stop Sampling in Continuous/Timer Triggered Mode
    inline void Stop() { LL_ADC_REG_StopConversion(ADC1);}

    // Enable Interrupt (End of Conversion only at the moment)
    void EnableIT();

    // Set Complete Callback
    void Attach(const std::function<void(void)> &cplt_cb)
    {
        cplt_callback_ = cplt_cb;
    }

    // Update ADC Bias
    inline void UpdateBias(int16_t bias)
    {
        bias_ = bias;
    }

    // Read Bias
    int16_t Bias() { return bias_; }

    // Set Low Pass Filter Parameter
    void SetFilter(float alpha)
    {
        filter_.SetAlpha(alpha);
        enable_filter_ = true;
    }

    // Read Current ADC Value
    inline uint16_t Read()
    {
        value_ = LL_ADC_REG_ReadConversionData12(ADC_) - bias_;

        if(enable_filter_) // Filter it?
            value_ = static_cast<uint16_t>(filter_.Filter(static_cast<float>(value_)));
        
        return value_;
    }

    // TODO: This should be up in a base "Peripheral Class"
    // Interrupt Setup
    // static void IRQ()
    // {
    //     uint32_t IRQn = SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk;
    //     ISR_VTABLE[IRQn]->ISR();
    // }

    // TODO: This should be up in a base "Peripheral Class"
    inline void ISR()
    {
        // DO ADC Stuff
        if (LL_ADC_IsActiveFlag_EOC(ADC_))
        {
            // Clear Flag
            LL_ADC_ClearFlag_EOC(ADC_);

            // Execute Callback
            cplt_callback_();
        }
    }

private:

    // STM32 ADC Type
    ADC_TypeDef *ADC_;
    
    // ADC Bias
    int16_t bias_;

    // Current ADC Sample Value
    uint16_t value_;

    // Low Pass Filter
    LowPassFilter filter_;

    // Low Pass Filter Enabled
    bool enable_filter_;

    // Interrupts Enabled?
    bool enable_interrupt_;

    // ISR Number
    IRQn_Type IRQn_; 

    // ISR Table
    //static ADCDevice* ISR_VTABLE[kMaxInterrupts];

    // Interrupt Callback
    std::function<void(void)> cplt_callback_ = [=](void) {};
};

#endif // CORE_PERIPHERAL_ADC_H_