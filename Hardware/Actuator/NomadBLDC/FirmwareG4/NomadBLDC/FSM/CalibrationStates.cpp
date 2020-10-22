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
#include <Utilities/math.h>

MeasureResistanceState::MeasureResistanceState() : NomadBLDCState("Measure Resistance", 3)
{
    kI_ = 10.0f;                          // [(V/s)/A]
}

void MeasureResistanceState::Run_(float dt)
{
    // Logger::Instance().Print("Startup Running\r\n");
    //LL_GPIO_SetOutputPin(USER_GPIO_GPIO_Port, USER_GPIO_Pin);

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



MeasureInductanceState::MeasureInductanceState() : NomadBLDCState("Measure Inductance", 4)
{
}

void MeasureInductanceState::Run_(float dt)
{
    // Logger::Instance().Print("Startup Running\r\n");
    //LL_GPIO_SetOutputPin(USER_GPIO_GPIO_Port, USER_GPIO_Pin);

    float voltage_high = motor->config_.calib_voltage;
    float voltage_low = -motor->config_.calib_voltage;

    float test_voltages[2] = {voltage_low, voltage_high};

    // Get Motor Ref
    Motor *motor = data_->controller->GetMotor();
    if (cycle_count_ < num_measure_cycles_) // Send Measure Signal
    {
        // for (int i = 0; i < 2; ++i)
        // TODO: Case Step_Low, Step_High etc

        if (step_id_++ == 0) // When you step you are reading the previous step.  TODO: Make this Better!
            I_alpha_[1] += motor->state_.I_a;
        else
            I_alpha_[0] += motor->state_.I_a;

        if (step_id_ == 2)
            step_id_ = 0;

        // Test voltage along phase A
        data_->controller->SetModulationOutput(0.0f, test_voltages[step_id_], 0.0f);

        return;
    }

    // Measurement is Complete. Process and Send Back
    float v_L = 0.5f * (voltage_high - voltage_low); // Inductor Voltage

    // di/dt
    float dI_by_dt = (I_alpha_[1] - I_alpha_[0]) / (dt * static_cast<float>(num_measure_cycles_));

    // Compute Inductance
    float L = v_L / dI_by_dt;
    
    // Update Measurement
    measurement_t measurement;
    measurement.f32 = L;

    // TODO: arbitrary values set for now
    if (L < 1e-6f || L > 500e-6f)
    {
        //Logger::Instance().Print("ERROR: Inductance Measurement Out of Range: %f\r\n", L);
        CommandHandler::SendMeasurementComplete(command_feedback_t::MEASURE_INDUCTANCE_COMPLETE, error_type_t::MEASUREMENT_OUT_OF_RANGE, measurement);
    }

    // PMSM D/Q Inductance are the same.
    motor->config_.phase_inductance_d = L;
    motor->config_.phase_inductance_q = L;

    // Logger::Instance().Print("Phase Inductance: %f Henries\r\n", L);
    CommandHandler::SendMeasurementComplete(command_feedback_t::MEASURE_INDUCTANCE_COMPLETE, error_type_t::SUCCESSFUL, measurement);

    // Set next state to idle
    data_->controller->SetControlMode(control_mode_type_t::IDLE_MODE);
}

void MeasureInductanceState::Enter_(uint32_t current_time)
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

    // Zero Current Alphas
    I_alpha_[0] = 0.0f;
    I_alpha_[1] = 0.0f;

    step_id_ = 0;

    // How Many Test Cycles?
    num_measure_cycles_ = static_cast<uint32_t>(3.0f / data_->controller->GetControlUpdatePeriod()); // Test runs for 3s
}

void MeasureInductanceState::Exit_(uint32_t current_time)
{
    // Enable Gate Driver
    data_->controller->GetGateDriver()->DisableDriver();

    // Turn On PWM Outputs
    data_->controller->EnablePWM(false);

    // Set Idle/"Zero" PWM
    data_->controller->SetDuty(0.5f, 0.5f, 0.5f);
}





MeasurePhaseOrderState::MeasurePhaseOrderState() : NomadBLDCState("Measure Phase Order", 5)
{
}

void MeasurePhaseOrderState::Run_(float dt)
{

    // Some Statics
    static float step_size = 1.0f / 5000.0f;
    static float scan_range = 8.0f * Core::Math::kPI;

    // Get Motor Ref
    Motor *motor = data_->controller->GetMotor();
    float test_voltage = motor->config_.calib_current * motor->config_.phase_resistance;

    // Check Current Mode
    switch (state_)
    {
    case (0):
        if (cycle_count_ < 2.0f * dt) // 2.0 Second Lock Duration
        {
            data_->controller->SetModulationOutput(0.0f, test_voltage, 0.0f);
        }
        else
        {
            state_ = 1;
            motor->Update(); // Update Rotor Position Reading

            // Update Start Theta Value
            theta_start_ = motor->state_.theta_mech_true;
        }
        break;
    case (1):
        if (reference_angle_ < scan_range)
        {
            // Set Modulation Output
            data_->controller->SetModulationOutput(reference_angle_, test_voltage, 0.0f);

            // Update State/Position Sensor
            motor->Update();

            // Update Reference Angle
            reference_angle_ += step_size;
        }
        else
        {
            theta_end_ = motor->state_.theta_mech_true;

            // Compute Phase Order
            if (theta_end_ - theta_start_ > 0)
            {
                motor->config_.phase_order = 1;
            }
            else
            {
                motor->config_.phase_order = 0;
            }

            // Feedback measurement
            measurement_t measurement;
            measurement.i32 = motor->config_.phase_order;

            // Send Complete Feedback
            CommandHandler::SendMeasurementComplete(command_feedback_t::MEASURE_PHASE_ORDER_COMPLETE, error_type_t::SUCCESSFUL, measurement);

            // Set next state to idle
            data_->controller->SetControlMode(control_mode_type_t::IDLE_MODE);
        }
        break;
    default:
        break;
    }
}

void MeasurePhaseOrderState::Enter_(uint32_t current_time)
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

    // Reset Calibration Variables
    reference_angle_ = 0.f;
    theta_start_ = 0.f;
    theta_end_ = 0.f;
    state_ = 0;

}

void MeasurePhaseOrderState::Exit_(uint32_t current_time)
{
    // Enable Gate Driver
    data_->controller->GetGateDriver()->DisableDriver();

    // Turn On PWM Outputs
    data_->controller->EnablePWM(false);

    // Set Idle/"Zero" PWM
    data_->controller->SetDuty(0.5f, 0.5f, 0.5f);
}
