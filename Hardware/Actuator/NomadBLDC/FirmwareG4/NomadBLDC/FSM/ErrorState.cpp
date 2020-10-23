/*
 * ErrorState.cpp
 *
 *  Created on: October 23, 2020
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
#include <FSM/ErrorState.h>

ErrorState::ErrorState() : NomadBLDCState("ERROR", 100)
{
}
void ErrorState::Run_(float dt)
{
    //Logger::Instance().Print("Idle Running\r\n");
    // Idle Task does Idle Things
}
void ErrorState::Enter_(uint32_t current_time)
{
    // Turn Status LED Off
    // TODO: Blink, etc
    LEDService::Instance().Off();

    // Disable Gate Driver
    data_->controller->GetGateDriver()->DisableDriver();

    // Turn Off PWM
    data_->controller->EnablePWM(false);

    // Reset Controller State
    data_->controller->Reset();

    Logger::Instance().Print("Entering Error State!!!\r\n");
}
