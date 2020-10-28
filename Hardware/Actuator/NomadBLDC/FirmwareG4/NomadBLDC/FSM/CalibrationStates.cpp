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

MeasureResistanceState::MeasureResistanceState() : NomadBLDCState(NomadBLDCStateID::STATE_CALIB_RESISTANCE)
{
}

void MeasureResistanceState::Run_(float dt)
{
    
    const static float kp_I = 10.0f; // [(V/s)/A]
    // Get Motor Ref
    Motor *motor = data_->controller->GetMotor();
    float alpha = 0.0f;
    if (cycle_count_ < num_measure_cycles_) // Send Measure Signal
    {
        float I_alpha = motor->state_.I_a;
        test_voltage_ += (kp_I * dt) * (motor->config_.calib_current - I_alpha);
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

    // Send Complete Signal
    CommandHandler::SendMeasurementComplete(command_feedback_t::MEASURE_RESISTANCE_COMPLETE, error_type_t::SUCCESSFUL, measurement);

    // Set next state to idle
    data_->controller->SetControlMode(control_mode_type_t::IDLE_MODE);
}

void MeasureResistanceState::Enter_(uint32_t current_time)
{
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
    // Enable Gate Driver
    data_->controller->GetGateDriver()->DisableDriver();

    // Turn On PWM Outputs
    data_->controller->EnablePWM(false);

    // Set Idle/"Zero" PWM
    data_->controller->SetDuty(0.5f, 0.5f, 0.5f);

}



MeasureInductanceState::MeasureInductanceState() : NomadBLDCState(NomadBLDCStateID::STATE_CALIB_INDUCTANCE)
{
}

void MeasureInductanceState::Run_(float dt)
{
    float voltage_high = motor->config_.calib_voltage;
    float voltage_low = -motor->config_.calib_voltage;

    float test_voltages[2] = {voltage_low, voltage_high};

    // Get Motor Ref
    Motor *motor = data_->controller->GetMotor();
    if (cycle_count_ < num_measure_cycles_) // Send Measure Signal
    {
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


MeasurePhaseOrderState::MeasurePhaseOrderState() : NomadBLDCState(NomadBLDCStateID::STATE_CALIB_PHASE_ORDER)
{
}

void MeasurePhaseOrderState::Run_(float dt)
{
    // Some Statics
    static float step_size = 1.0f / 5000.0f;
    static float scan_range = 4.0f * Core::Math::k2PI;

    // Get Motor Ref
    Motor *motor = data_->controller->GetMotor();
    float test_voltage = motor->config_.calib_current * motor->config_.phase_resistance;

    // Check Current Mode
    switch (state_)
    {
    case (state_t::LOCK_ROTOR):
        if (cycle_count_ < 2.0f / dt) // 2.0 Second Lock Duration
        {
            data_->controller->SetModulationOutput(0.0f, test_voltage, 0.0f);
        }
        else
        {
            state_ = MEASURE_PHASE_ORDER;
            motor->Update(); // Update Rotor Position Reading

            // Update Start Theta Value
            theta_start_ = motor->state_.theta_mech_true;
        }
        break;
    case (state_t::MEASURE_PHASE_ORDER):
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
            // Update End Theta Value
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

            // Compute Pole Pairs
            float total_theta_mech = std::abs(theta_end_ - theta_start_);
            uint16_t pole_pairs = static_cast<uint16_t>(scan_range/total_theta_mech);

            // Pole pairs should be evenly divisible by 3(3 phases)
            if(pole_pairs % 3 != 0)
            {
                // If not.  We have some error but should be close enough to tick +/- to find it
                pole_pairs = (pole_pairs+1) % 3 == 0 ? pole_pairs + 1 : pole_pairs - 1;
            }

            // Update Config
            motor->config_.num_pole_pairs = pole_pairs;
            motor->PositionSensor()->SetPolePairs(pole_pairs);

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

    // Initial State
    state_ = state_t::LOCK_ROTOR; 
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

MeasureEncoderOffsetState::MeasureEncoderOffsetState() : NomadBLDCState(NomadBLDCStateID::STATE_CALIB_ENCODER)
{
}

void MeasureEncoderOffsetState::Setup()
{
    Motor *motor = data_->controller->GetMotor();

    // Init Tables
    window_size_ = 128;

    // Samples per mechanical rotation.  Multiple of Pole Pairs for filtering
    num_samples_ = window_size_ * motor->config_.num_pole_pairs;  

    // Sub sampling for smoothing         
    num_sub_samples_ = 160;                                             

    error_forward_ = new float[num_samples_];
    error_backward_ = new float[num_samples_];

    error_ = new float[num_samples_];
    error_filtered_ = new float[num_samples_];

    // Zero Array.  Do this explicitly in case compilers vary
    memset(error_forward_, 0, sizeof(float)*num_samples_);
    memset(error_backward_, 0, sizeof(float)*num_samples_);
    memset(error_, 0, sizeof(float)*num_samples_);
    memset(error_filtered_, 0, sizeof(float)*num_samples_);

    LUT_ = new int8_t[kLUTSize]; // Clear the previous lookup table.

    // Zero Array.  Do this explicitly in case compilers vary
    memset(LUT_, 0, sizeof(int8_t)*kLUTSize);
    
    raw_forward_ = new int32_t[num_samples_];
    raw_backward_ = new int32_t[num_samples_];

    // Zero Array.  Do this explicitly in case compilers vary
    memset(raw_forward_, 0, sizeof(int32_t)*num_samples_);
    memset(raw_backward_, 0, sizeof(int32_t)*num_samples_);
    
}
void MeasureEncoderOffsetState::Run_(float dt)
{
    static int32_t sample_idx = 0;
    static int32_t subsample_idx = 0;
    
    // delta angle between samples
    float delta_sample = 2.0f * Core::Math::kPI * motor_->config_.num_pole_pairs / (num_samples_ * num_sub_samples_); 
    
    // Check Current Mode
    switch (state_)
    {
    case (state_t::LOCK_ROTOR):
        if (cycle_count_ < (2.0f / dt)) // 2.0 Second Lock Duration
        {
            data_->controller->SetModulationOutput(0.0f, test_voltage_, 0.0f);
        }
        else
        {   
            // Reset Sample Index
            sample_idx = 0; 

            // Sample new rotor positipm
            motor_->Update(); 

            // Set Next State
            state_ = state_t::CALIBRATE_FORWARD;
        }
        break;
    case (state_t::CALIBRATE_FORWARD):
        if (sample_idx < num_samples_)
        {
            if(subsample_idx++ < num_sub_samples_)
            {
                // Update Reference Angle
                reference_angle_ += delta_sample;

                // Set Modulation Output
                data_->controller->SetModulationOutput(reference_angle_, test_voltage_, 0.0f);

                // Update State/Position Sensor
                motor_->Update();
            }
            else
            {
                // Update State/Position Sensor
                motor_->Update(); 

                // Get Mechanical Position
                theta_actual_ = motor_->PositionSensor()->GetMechanicalPositionTrue();

                // Update Error Table and Encoder Count Table
                error_forward_[sample_idx] = reference_angle_ / motor_->config_.num_pole_pairs - theta_actual_;
                raw_forward_[sample_idx] = motor_->PositionSensor()->GetRawPosition();

                sample_idx++;
                subsample_idx = 0;
            }
        }
        else
        {
            // Reset
            sample_idx = 0;
            subsample_idx = 0;
            
            // Set Next State
            state_ = state_t::CALIBRATE_BACKWARD;
        }
        break;

    case (state_t::CALIBRATE_BACKWARD):
        if (sample_idx < num_samples_)
        {
            if (subsample_idx++ < num_sub_samples_)
            {
                // Update Reference Angle
                reference_angle_ -= delta_sample;

                // Set Modulation Output
                data_->controller->SetModulationOutput(reference_angle_, test_voltage_, 0.0f);

                // Update State/Position Sensor
                motor_->Update();
            }
            else
            {
                // Update State/Position Sensor
                motor_->Update();

                // Get Mechanical Position
                theta_actual_ = motor_->PositionSensor()->GetMechanicalPositionTrue();

                // Update Error Table and Encoder Count Table
                error_backward_[sample_idx] = reference_angle_ / motor_->config_.num_pole_pairs - theta_actual_;
                raw_backward_[sample_idx] = motor_->PositionSensor()->GetRawPosition();

                sample_idx++;
                subsample_idx = 0;
            }
        }
        else
        {
            // Reset
            sample_idx = 0;
            subsample_idx = 0;
            
            // Compute Offset and LUT
            ComputeOffsetLUT();

            // Build Measurement
            measurement_t elec_offset;
            elec_offset.f32 = motor_->PositionSensor()->GetElectricalOffset();

            // Send Complete Feedback
            CommandHandler::SendMeasurementComplete(command_feedback_t::MEASURE_ENCODER_OFFSET_COMPLETE, error_type_t::SUCCESSFUL, elec_offset);

            // Set next state to idle
            data_->controller->SetControlMode(control_mode_type_t::IDLE_MODE);
        }
        break;

    default:
        break;
    }
}

void MeasureEncoderOffsetState::Enter_(uint32_t current_time)
{
    // Turn Status LED On
    LEDService::Instance().On();

    // Enable Gate Driver
    data_->controller->GetGateDriver()->EnableDriver();

    // Turn On PWM Outputs
    data_->controller->EnablePWM(true);

    // Set Idle/"Zero" PWM
    data_->controller->SetDuty(0.5f, 0.5f, 0.5f);

    // Get Motor Ref
    motor_ = data_->controller->GetMotor();

    // Update Test Voltage
    test_voltage_ = motor_->config_.calib_current * motor_->config_.phase_resistance;

    // Reset Calibration Variables
    reference_angle_ = 0.f;
    theta_actual_ = 0.f;

    // Reset Encoder Calibration
    motor_->PositionSensor()->Reset();

    // Initial State
    state_ = state_t::LOCK_ROTOR; 
}

void MeasureEncoderOffsetState::Exit_(uint32_t current_time)
{
    // Disable Gate Driver
    data_->controller->GetGateDriver()->DisableDriver();

    // Turn On PWM Outputs
    data_->controller->EnablePWM(false);

    // Set Idle/"Zero" PWM
    data_->controller->SetDuty(0.5f, 0.5f, 0.5f);

    // Clear Memory
    delete[] error_forward_; 
    delete[] error_backward_;
    delete[] LUT_;
    delete[] raw_forward_;
    delete[] raw_backward_;
    delete[] error_;
    delete[] error_filtered_;
}

void MeasureEncoderOffsetState::ComputeOffsetLUT()
{
    // Compute Electrical Offset
    float offset = 0;
    for (int32_t i = 0; i < num_samples_; i++)
    {
        offset += (error_forward_[i] + error_backward_[num_samples_ - 1 - i]) / (2.0f * num_samples_); // calclate average position sensor offset
    }
    offset = fmod(offset * motor_->config_.num_pole_pairs, Core::Math::k2PI); // convert mechanical angle to electrical angle
    
    while(offset < 0) // Keep offset 0 to 2*PI
        offset += Core::Math::k2PI;

    // Update Offset
    motor_->PositionSensor()->SetElectricalOffset(offset); // Set Offset

    // Perform filtering to linearize position sensor eccentricity
    // FIR n-sample average, where n = number of samples in one electrical cycle
    // This filter has zero gain at electrical frequency and all integer multiples
    // So cogging effects should be completely filtered out.
    float mean = 0;

    // Average Forward and Backward Directions
    for (int32_t i = 0; i < num_samples_; i++)
    {
       error_[i] = 0.5f * (error_forward_[i] + error_backward_[num_samples_ - i - 1]);
    }

    for (int32_t i = 0; i < num_samples_; i++)
    {
        for (int32_t j = 0; j < window_size_; j++)
        {
            int32_t index = -window_size_ / 2 + j + i; // Indices from -window/2 to + window/2
            if (index < 0)
            {
                index += num_samples_;
            } // Moving average wraps around
            else if (index > num_samples_ - 1)
            {
                index -= num_samples_;
            }
            error_filtered_[i] += error_[index] / static_cast<float>(window_size_);
            
        }
        mean += error_filtered_[i] / num_samples_;
    }

    int32_t raw_offset = (raw_forward_[0] + raw_backward_[num_samples_ - 1]) / 2; //Insensitive to errors in this direction, so 2 points is plenty

    // Build our Lookup Table
    for (int32_t i = 0; i < kLUTSize; i++)
    {
        int32_t index = (raw_offset >> 7) + i;
        if (index > (kLUTSize - 1))
        {
            index -= kLUTSize;
        }  
        LUT_[index] = static_cast<int8_t>((error_filtered_[i * motor_->config_.num_pole_pairs] - mean) * static_cast<float>(motor_->PositionSensor()->GetCPR()) / Core::Math::k2PI);
    }
    motor_->PositionSensor()->SetOffsetLUT(LUT_); // Write Compensated Lookup Table
    return;
}