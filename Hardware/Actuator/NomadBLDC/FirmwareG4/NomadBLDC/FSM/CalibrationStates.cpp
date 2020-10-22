/*
 * CalibrationState.cpp
 *
 *  Created on: October 21, 2020
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
#include <FSM/CalibrationStates.h>

MeasureResistanceState::MeasureResistanceState() : NomadBLDCState("Measure Resistance", 3)
{
    kI_ = 10.0f;                          // [(V/s)/A]
}

void MeasureResistanceState::Run_(float dt)
{
    // Logger::Instance().Print("Startup Running\r\n");
    LL_GPIO_SetOutputPin(USER_GPIO_GPIO_Port, USER_GPIO_Pin);

    // Get Motor Ref
    Motor *motor = data_->controller->GetMotor();
    float alpha = 0.0f;
    if (cycle_count_ < num_measure_cycles_) // Send Measure Signal
    {
        float I_alpha = motor->state_.I_a;
        test_voltage_ += (kI_ * dt) * (motor->config_.calib_current - I_alpha);
        alpha = std::max(alpha,I_alpha);
        if (test_voltage_ > motor->config_.calib_voltage)
            test_voltage_ = motor->config_.calib_voltage; // Clamp Voltage
        if (test_voltage_ < -motor->config_.calib_voltage)
            test_voltage_ = -motor->config_.calib_voltage; // Clamp Voltage

        // Test voltage along phase A
        data_->controller->SetModulationOutput(0.0f, test_voltage_, 0.0f);

        
        return;
    }
//Logger::Instance().Print("MAX VOLTAGE: %f\r\n", motor->config_.calib_current);

    // Measurement is Complete. Process and Send Back
    // R = V/I
    float R = test_voltage_ / motor->config_.calib_current; 

    // Hit Cycle Count.  Now Compute Resistance
    measurement_t measurement;
    measurement.f32 = R;

    // Update Motor Config Value
    motor->config_.phase_resistance = R;

    if (fabs(test_voltage_) == fabs(motor->config_.calib_voltage) || R < 0.01f || R > 1.0f)
    {
        //motor->error = ERROR_PHASE_RESISTANCE_OUT_OF_RANGE;
       // Logger::Instance().Print("ERROR: Resistance Measurement Out of Range: %f\r\n", R);

        // Update Command Interface
        CommandHandler::SendMeasurementComplete(command_feedback_t::MEASURE_RESISTANCE_COMPLETE, error_type_t::MEASUREMENT_OUT_OF_RANGE, measurement);
    }
    
    Logger::Instance().Print("Phase Resistance: %f ohms\r\n", motor->config_.phase_resistance);

    // Send Complete Signal
    CommandHandler::SendMeasurementComplete(command_feedback_t::MEASURE_RESISTANCE_COMPLETE, error_type_t::SUCCESSFUL, measurement);

    // Set next state to idle
    data_->controller->SetControlMode(control_mode_type_t::IDLE_MODE);
}

void MeasureResistanceState::Enter_(uint32_t current_time)
{
    //Logger::Instance().Print("Entering Measure Resistance State.\r\n");

    // Turn Status LED On
    LEDService::Instance().On();

    // Enable Gate Driver
    data_->controller->GetGateDriver()->EnableDriver();

    // Turn On PWM Outputs
    data_->controller->EnablePWM(true);

    // Set Idle/"Zero" PWM
    data_->controller->SetDuty(0.5f, 0.5f, 0.5f);

    // Zero Test Voltage
    test_voltage_ = 0.0f;

    // How Many Test Cycles?
    num_measure_cycles_ = static_cast<uint32_t>(3.0f / data_->controller->GetControlUpdatePeriod()); // Test runs for 3s
}

void MeasureResistanceState::Exit_(uint32_t current_time)
{
   // Logger::Instance().Print("Exiting Measure Resistance State.\r\n");

    // Enable Gate Driver
    data_->controller->GetGateDriver()->DisableDriver();

    // Turn On PWM Outputs
    data_->controller->EnablePWM(false);

    // Set Idle/"Zero" PWM
    data_->controller->SetDuty(0.5f, 0.5f, 0.5f);

}
