
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

ADCDevice::ADCDevice(ADC_TypeDef *ADC) : ADC_(ADC)
{
}


// // Interrupts.  TODO: Make this dynamic so we can "register these"
// /**
//   * @brief This function handles ADC3 global interrupt.
//   */
// extern "C" void ADC3_IRQHandler(void)
// {

//     if (LL_ADC_IsActiveFlag_EOC(ADC1))
//     {
//         /* Clear flag ADC group regular end of unitary conversion */
//         LL_ADC_ClearFlag_EOC(ADC1);
//     }

//     if (LL_ADC_IsActiveFlag_EOC(ADC2))
//     {
//         /* Clear flag ADC group regular end of unitary conversion */
//         LL_ADC_ClearFlag_EOC(ADC2);
//     }

//     if (LL_ADC_IsActiveFlag_EOC(ADC3))
//     {
//         /* Clear flag ADC group regular end of unitary conversion */
//         LL_ADC_ClearFlag_EOC(ADC3);

//         current_measurement_cb();
//     }
// }
