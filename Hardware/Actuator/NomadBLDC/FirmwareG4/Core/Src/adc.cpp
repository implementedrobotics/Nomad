
/*
 * adc.cpp
 *
 *  Created on: September 21, 2020
 *      Author: Quincy Jones
 *
 * Copyright (c) <2020> <Quincy Jones - quincy@implementedrobotics.com/>
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
#include <Peripherals/adc.h>

// C System Files

// C++ System Files

// Project Includes

//ADCDevice* ADCDevice::ISR_VTABLE[kMaxInterrupts];
static ADCDevice* g_ISR_VTABLE[ADCDevice::kMaxInterrupts];

ADCDevice::ADCDevice(ADC_TypeDef *ADC) : ADC_(ADC), bias_(0), value_(0), enable_filter_(false), enable_interrupt_(false)
{
}

// Enable ADC
void ADCDevice::Enable()
{
    __IO uint32_t wait_loop_index = 0U;
    if (LL_ADC_IsEnabled(ADC_) == 0) // Is ADC Alread Enabled?  If so bail
    {
        /* Disable ADC deep power down (enabled by default after reset state) */
        LL_ADC_DisableDeepPowerDown(ADC_);

        /* Enable ADC internal voltage regulator */
        LL_ADC_EnableInternalRegulator(ADC_);

        /* Delay for ADC internal voltage regulator stabilization.                */
        /* Compute number of CPU cycles to wait for, from delay in us.            */
        /* Note: Variable divided by 2 to compensate partially                    */
        /*       CPU processing cycles (depends on compilation optimization).     */
        /* Note: If system core clock frequency is below 200kHz, wait time        */
        /*       is only a few CPU processing cycles.                             */
        wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
        while (wait_loop_index != 0)
        {
            wait_loop_index--;
        }

        /* Run ADC self calibration */
        LL_ADC_StartCalibration(ADC_, LL_ADC_SINGLE_ENDED);

        while (LL_ADC_IsCalibrationOnGoing(ADC_) != 0); // Wait for Calibration

        /* Delay between ADC end of calibration and ADC enable.                   */
        /* Note: Variable divided by 2 to compensate partially                    */
        /*       CPU processing cycles (depends on compilation optimization).     */
        /* This can be optimized.  In no hurray for our application.  Enable is not time critical. */
        wait_loop_index = ((LL_ADC_DELAY_CALIB_ENABLE_ADC_CYCLES * 64) >> 1);
        while (wait_loop_index != 0)
        {
            wait_loop_index--;
        }

        /* Enable ADC */
        LL_ADC_Enable(ADC_);

        /* Poll for ADC ready to convert */
        while (LL_ADC_IsActiveFlag_ADRDY(ADC_) == 0);

        /* Note: ADC flag ADRDY is not cleared here to be able to check ADC       */
        /*       status afterwards.                                               */
        /*       This flag should be cleared at ADC Deactivation, before a new    */
        /*       ADC activation, using function "LL_ADC_ClearFlag_ADRDY()".       */
    }
}

uint16_t ADCDevice::Sample()
{
    if ((LL_ADC_IsEnabled(ADC_) == 1) &&
        (LL_ADC_IsDisableOngoing(ADC_) == 0) &&
        (LL_ADC_REG_IsConversionOngoing(ADC_) == 0))
    {
        LL_ADC_REG_StartConversion(ADC_);
    }
    else
    {
        /* Error: ADC conversion start could not be performed */
        return 0;
    }

    // Wait on Completed Conversion
    while (LL_ADC_IsActiveFlag_EOC(ADC_) == 0);

    // Clear Flag.  Not Strictly needed here as the ReadConversion clears
    LL_ADC_ClearFlag_EOC(ADC_);

    return Read();
}

void ADCDevice::EnableIT()
{
    // Find IRQ Number
    if(ADC_ == ADC1 || ADC_ == ADC2) // TODO: Shared Interrupt actually needs a bit more work...
    {
        IRQn_ = ADC1_2_IRQn;
    }
    else if(ADC_ == ADC3)
    {
        IRQn_ = ADC3_IRQn;
    }
    else if(ADC_ == ADC4)
    {
        IRQn_ = ADC4_IRQn;
    }
    else if(ADC_ == ADC5)
    {
        IRQn_ = ADC5_IRQn;
    }
    else // Invalid
    {
        return;
    }

    // Make sure IRQ is enabled
    // TODO: Priority...
    NVIC_SetPriority(IRQn_, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
    NVIC_EnableIRQ(IRQn_);

    // Update ISR Table
    g_ISR_VTABLE[IRQn_] = this;

    enable_interrupt_ = true;
    LL_ADC_EnableIT_EOC(ADC_);

    // Not Sure Why Dynamic Does not work
    // __disable_irq();
    // NVIC_SetVector(IRQn, (uint32_t)&IRQ);
    // __enable_irq();
}



// Interrupts
extern "C" void ADC1_2_IRQHandler(void)
{
    g_ISR_VTABLE[ADC1_2_IRQn]->ISR();
}

extern "C" void ADC3_IRQHandler(void)
{
    LL_GPIO_SetOutputPin(USER_GPIO_GPIO_Port, USER_GPIO_Pin);
    g_ISR_VTABLE[ADC3_IRQn]->ISR();
    LL_GPIO_ResetOutputPin(USER_GPIO_GPIO_Port, USER_GPIO_Pin);
}

extern "C" void ADC4_IRQHandler(void)
{
    g_ISR_VTABLE[ADC4_IRQn]->ISR();    
}

extern "C" void ADC5_IRQHandler(void)
{
    g_ISR_VTABLE[ADC5_IRQn]->ISR();    
}

