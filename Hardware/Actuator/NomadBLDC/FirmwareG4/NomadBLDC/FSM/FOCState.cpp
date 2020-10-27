/*
 * FOCState.cpp
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
#include <FSM/FOCState.h>
#include <Utilities/math.h>

FOCState::FOCState() : NomadBLDCState(NomadBLDCStateID::STATE_FOC)
{
}

void FOCState::Run_(float dt)
{
    // Enter Critical Section
    __disable_irq();

    // DWT->CYCCNT = 0;
    // Get Motor Ref
    Motor *motor = data_->controller->GetMotor();
    MotorController *controller = data_->controller;
    switch (data_->controller->GetControlMode())
    {
        case (control_mode_type_t::FOC_VOLTAGE_MODE):
        {
            controller->state_.V_d = controller->state_.V_d_ref;
            controller->state_.V_q = controller->state_.V_q_ref;

            // TODO: Voltage Limit for I_rms.  V_max = I_max * Phase_R -> Limit Norm/Modulus

            controller->SetModulationOutput(motor->state_.theta_elec, controller->state_.V_d_ref, controller->state_.V_q_ref);

            // Update V_d/V_q
            // TODO: Should probably have this more universal somewhere
            controller->dq0(motor->state_.theta_elec, motor->state_.I_a, motor->state_.I_b, motor->state_.I_c, &controller->state_.I_d, &controller->state_.I_q); //dq0 transform on currents
            controller->state_.V_d = controller->state_.I_d * motor->config_.phase_resistance;
            controller->state_.V_q = controller->state_.I_q * motor->config_.phase_resistance;
            break;
        }
        case (control_mode_type_t::FOC_CURRENT_MODE):
        {
            controller->CurrentControl();
            break;
        }
        case (control_mode_type_t::FOC_TORQUE_MODE):
        {
            controller->TorqueControl();
            break;
        }
        default:
            break;
    }
    // uint32_t span = DWT->CYCCNT;
    //Logger::Instance().Print("Count: %f\r\n", j);
    // Exit Critical
    __enable_irq();
}

void FOCState::Enter_(uint32_t current_time)
{
    // Check Motor Calibribration
    if (!data_->controller->GetMotor()->config_.calibrated)
    {
        Logger::Instance().Print("[FOCVoltageState]: ERROR: Motor NOT Calibrated!\r\nPlease calibrate and save a valid configuration.\r\n");
        data_->controller->SetControlMode(control_mode_type_t::ERROR_MODE);
        return;
    }

    // Turn Status LED On
    LEDService::Instance().On();

    // Enable Gate Driver
    data_->controller->GetGateDriver()->EnableDriver();

    // Turn On PWM Outputs
    data_->controller->EnablePWM(true);

    // Set Idle/"Zero" PWM
    data_->controller->SetDuty(0.5f, 0.5f, 0.5f);

    // Reset Controller Setpoints/Data
    data_->controller->Reset();
}

void FOCState::Exit_(uint32_t current_time)
{
   // Logger::Instance().Print("Exiting Measure Resistance State.\r\n");

    // Enable Gate Driver
    data_->controller->GetGateDriver()->DisableDriver();

    // Turn On PWM Outputs
    data_->controller->EnablePWM(false);

    // Set Idle/"Zero" PWM
    data_->controller->SetDuty(0.5f, 0.5f, 0.5f);

    // Reset Controller Setpoints/Data
    data_->controller->Reset();
}
