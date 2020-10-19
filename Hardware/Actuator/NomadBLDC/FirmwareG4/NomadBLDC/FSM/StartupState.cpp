/*
 * StartupState.cpp
 *
 *  Created on: October 18, 2020
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
 */

// C System Files

// C++ System Files

// Third Party Includes

// Project Include Files
#include <Logger.h>
#include <FSM/StartupState.h>

StartupState::StartupState() : NomadBLDCState("Startup", 0)
{
    num_adc_calib_samples_ = 1024;
}

void StartupState::Run_(float dt)
{
    //Logger::Instance().Print("Startup Running\r\n");
    LL_GPIO_SetOutputPin(USER_GPIO_GPIO_Port, USER_GPIO_Pin);

    if (cycle_count_ < num_adc_calib_samples_)
    {
        adc3_offset_ += LL_ADC_REG_ReadConversionData12(ADC3);
        adc2_offset_ += LL_ADC_REG_ReadConversionData12(ADC2);
        adc1_offset_ += LL_ADC_REG_ReadConversionData12(ADC1);
        return;
    }


    // Hit Calibration Count.  Compute Average and Return
    adc1_offset_ = adc1_offset_ / num_adc_calib_samples_;
    adc2_offset_ = adc2_offset_ / num_adc_calib_samples_;
    adc3_offset_ = adc3_offset_ / num_adc_calib_samples_;

    // TODO: Set Offset on Motor Controller

    Logger::Instance().Print("\r\nADC OFFSET: %d and %d and %d\r\n", adc1_offset_, adc2_offset_, adc3_offset_);

    // Set mode to idle
    data_->control_mode = control_mode_type_t::IDLE_MODE;
}
void StartupState::Enter_(uint32_t current_time)
{
   Logger::Instance().Print("Entering Startup State!!!\r\n");

   // Set Idle/"Zero" PWM
   //motor_controller->SetDuty(0.5f, 0.5f, 0.5f);

   // Zero Offsets
   adc1_offset_ = 0;
   adc2_offset_ = 0;
   adc3_offset_ = 0;
}

void StartupState::Exit_(uint32_t current_time)
{
   Logger::Instance().Print("Exiting Startup State!!!\r\n");
}

