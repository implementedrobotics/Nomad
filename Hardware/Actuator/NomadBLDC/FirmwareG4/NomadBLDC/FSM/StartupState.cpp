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
#include <Peripherals/adc.h>
#include <FSM/StartupState.h>

StartupState::StartupState() : NomadBLDCState(NomadBLDCStateID::STATE_STARTUP)
{
    num_adc_calib_samples_ = 1024;
}

void StartupState::Run_(float dt)
{
    if (cycle_count_ < num_adc_calib_samples_)
    {
        adc1_offset_ += data_->controller->GetADC1()->Read();
        adc2_offset_ += data_->controller->GetADC2()->Read();
        adc3_offset_ += data_->controller->GetADC3()->Read();

        //Logger::Instance().Print(":%d\r\n", adc1_offset_);

        return;
    }

    // Hit Calibration Count.  Compute Average and Return
    adc1_offset_ = adc1_offset_ / num_adc_calib_samples_;
    adc2_offset_ = adc2_offset_ / num_adc_calib_samples_;
    adc3_offset_ = adc3_offset_ / num_adc_calib_samples_;

    // Set Offset on Motor Controller
    data_->controller->GetADC1()->UpdateBias(adc1_offset_);
    data_->controller->GetADC2()->UpdateBias(adc2_offset_);
    data_->controller->GetADC3()->UpdateBias(adc3_offset_);

    // Set mode to idle
    data_->controller->SetControlMode(control_mode_type_t::IDLE_MODE);
}

void StartupState::Enter_(uint32_t current_time)
{
    // Turn Status LED On
    LEDService::Instance().On();

    // Enable Gate Driver
    data_->controller->GetGateDriver()->EnableDriver();

    // Turn On PWM Outputs
    data_->controller->EnablePWM(true);

    // Set Idle/"Zero" PWM
    data_->controller->SetDuty(0.5f, 0.5f, 0.5f);

    // Zero Offsets
    adc1_offset_ = 0;
    adc2_offset_ = 0;
    adc3_offset_ = 0;
}

void StartupState::Exit_(uint32_t current_time)
{
    // Disable Gate Driver
    data_->controller->GetGateDriver()->DisableDriver();

    // Turn Off PWM
    data_->controller->EnablePWM(false);

    // Reset Controller State
    data_->controller->Reset();
}
