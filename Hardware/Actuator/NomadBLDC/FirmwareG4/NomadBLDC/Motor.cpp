/*
 * Motor.cpp
 *
 *  Created on: August 26, 2019
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

// Primary Include
#include "Motor.h"

// C System Files

// C++ System Files

// Project Includes
#include "MotorController.h"
#include "Logger.h"
#include "math_ops.h"
#include <Utilities/utils.h>

Motor::Motor(float sample_time, float K_v, uint32_t pole_pairs) : sample_time_(sample_time),
                                                                  dirty_(false)
{
    // Zero State
    memset(&state_, 0, sizeof(state_));

    // Setup Default Configs
    config_.num_pole_pairs = pole_pairs;
    config_.continuous_current_max = 20.0f; 
    config_.continuous_current_tau = 60.0f;
    config_.phase_resistance = 0.0f;
    config_.phase_inductance_d = 0.0f;
    config_.phase_inductance_q = 0.0f;
    config_.phase_order = 1;
    config_.calib_current = 15.0f;
    config_.calib_voltage = 3.0f;
    config_.gear_ratio = 1.0f; // No Gearbox by default
    config_.calibrated = 0;

    // Update KV Calulations
    SetKV(K_v);

    // Setup Position Sensor
    rotor_sensor_ = new PositionSensorAS5x47(sample_time_, config_.num_pole_pairs);
}

void Motor::SetSampleTime(float sample_time)
{
    sample_time_ = sample_time;
    rotor_sensor_->SetSampleTime(sample_time);
}

void Motor::Update()
{
    // Update Position Sensor
    rotor_sensor_->Update(sample_time_);

    // Update State
    state_.theta_mech = rotor_sensor_->GetMechanicalPosition();
    state_.theta_mech_true = rotor_sensor_->GetMechanicalPositionTrue();
    state_.theta_mech_dot = rotor_sensor_->GetMechanicalVelocity();
    state_.theta_elec = rotor_sensor_->GetElectricalPosition();
    state_.theta_elec_dot = rotor_sensor_->GetElectricalVelocity();

    // Update Temparature Observer
}

void Motor::PrintPosition()
{
    //printf(" Mechanical Angle:  %f/%f/%f    Electrical Angle:  %f    Raw:  %ld\n\r", state_.theta_mech, state_.theta_mech_true, rotor_sensor_->config_.offset_mech, state_.theta_elec, rotor_sensor_->GetRawPosition());
}
void Motor::ZeroOutputPosition()
{
    Update(); // Make sure we are updated
    rotor_sensor_->ZeroPosition();
    Update(); // Post update
}
bool Motor::Calibrate(MotorController *controller)
{

    controller->SetDuty(0.5f, 0.5f, 0.5f); // Make sure we have no PWM period

    // TODO: Check Error Here
    // Measure Resistance
    MeasureMotorResistance(controller, config_.calib_current, config_.calib_voltage);

    controller->SetDuty(0.5f, 0.5f, 0.5f); // Make sure we have no PWM period

    osDelay(1);

    // AND Here
    // Measure Inductance
    MeasureMotorInductance(controller, -config_.calib_voltage, config_.calib_voltage);

    controller->SetDuty(0.5f, 0.5f, 0.5f); // Make sure we have no PWM period

    // Order Phases
    OrderPhases(controller);

    controller->SetDuty(0.5f, 0.5f, 0.5f); // Make sure we have no PWM period

    //printf("\r\nCooling Down 3s...\r\n");
    osDelay(3);
    
    // Offset Calibration
    CalibrateEncoderOffset(controller);

    config_.calibrated = 1; // Update Flag
    dirty_ = true;

    return true;
}

bool Motor::MeasureMotorInductance(MotorController *controller, float voltage_low, float voltage_high)
{
    //printf("\n\rMeasure Motor Inductance...\n\r");

    float test_voltages[2] = {voltage_low, voltage_high};
    float I_alpha[2] = {0.0f};

    static const int num_cycles = 3.0f / sample_time_; // Test runs for 3s;

    measurement_t measurement;
    measurement.f32 = 0.0f;

    Logger::Instance().Print("[MOTOR] Measure Motor Inductance...\n");
    // Shutdown Phases
    controller->SetDuty(0.5f, 0.5f, 0.5f);

    for (int t = 0; t < num_cycles; ++t)
    {
        for (int i = 0; i < 2; ++i)
        {
            uint32_t ret = osThreadFlagsWait(CURRENT_MEASUREMENT_COMPLETE_SIGNAL, osFlagsWaitAny, CURRENT_MEASUREMENT_TIMEOUT);
            if (ret != 0)
            {
                // motor->error = ERROR_PHASE_INDUCTANCE_MEASUREMENT_TIMEOUT;
                //printf("ERROR: Phase Inductance Measurement Timeout\n\r");
                CommandHandler::SendMeasurementComplete(command_feedback_t::MEASURE_INDUCTANCE_COMPLETE, error_type_t::MEASUREMENT_TIMEOUT, measurement);
                Logger::Instance().Print("ERROR: Phase Inductance Measurement Timeout\n");
                return false;
            }
            if (i == 1) // When you step you are reading the previous step.  TODO: Make this Better!
                I_alpha[0] += state_.I_a;
            else
                I_alpha[1] += state_.I_a;

            // Test voltage along phase A
            controller->SetModulationOutput(0.0f, test_voltages[i], 0.0f);
        }
    }

    // Shutdown phases
    controller->SetDuty(0.5f, 0.5f, 0.5f);
    //printf("I Alpha: %f/%f\n\r", I_alpha[0], I_alpha[1]);
    float v_L = 0.5f * (voltage_high - voltage_low); // Inductor Voltage

    // Note: A more correct formula would also take into account that there is a finite timestep.
    // However, the discretisation in the current control loop inverts the same discrepancy
    float dI_by_dt = (I_alpha[1] - I_alpha[0]) / (sample_time_ * (float)num_cycles);
    float L = v_L / dI_by_dt;
    measurement.f32 = L;
    // TODO: arbitrary values set for now
    if (L < 1e-6f || L > 500e-6f)
    {
        //motor->error = ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE;
        //printf("ERROR: Inductance Measurement Out of Range: %f\n\r", L);
        //Logger::Instance().Print("Phase Inductance: %f Henries\n", L);
        Logger::Instance().Print("ERROR: Inductance Measurement Out of Range: %f\n\r", L);
        CommandHandler::SendMeasurementComplete(command_feedback_t::MEASURE_INDUCTANCE_COMPLETE, error_type_t::MEASUREMENT_OUT_OF_RANGE, measurement);
        return false;
    }

    // PMSM D/Q Inductance are the same.
    config_.phase_inductance_d = L;
    config_.phase_inductance_q = L;

    Logger::Instance().Print("Phase Inductance: %f Henries\n", L);
    CommandHandler::SendMeasurementComplete(command_feedback_t::MEASURE_INDUCTANCE_COMPLETE, error_type_t::SUCCESSFUL, measurement);
    return true;
}

bool Motor::MeasureMotorResistance(MotorController *controller, float test_current, float max_voltage)
{
    static const float kI = 10.0f;                          // [(V/s)/A]
    static const int num_test_cycles = 3.0f / sample_time_; // Test runs for 3s
    float test_voltage = 0.0f;

    measurement_t measurement;
    measurement.f32 = 0.0f;


    Logger::Instance().Print("[MOTOR] Measure Motor Resistance...\n");
    controller->SetDuty(0.5f, 0.5f, 0.5f); // Make sure we have no PWM period


    for (int i = 0; i < num_test_cycles; ++i)
    {
        //osEvent evt = osSignalWait(CURRENT_MEASUREMENT_COMPLETE_SIGNAL, CURRENT_MEASUREMENT_TIMEOUT);
        uint32_t ret = osThreadFlagsWait(CURRENT_MEASUREMENT_COMPLETE_SIGNAL, osFlagsWaitAny, CURRENT_MEASUREMENT_TIMEOUT);
        if (ret != 0)
        {
            // motor->error = ERROR_PHASE_RESISTANCE_MEASUREMENT_TIMEOUT;
            // Update Serial Interface
            CommandHandler::SendMeasurementComplete(command_feedback_t::MEASURE_RESISTANCE_COMPLETE, error_type_t::MEASUREMENT_TIMEOUT, measurement);
            Logger::Instance().Print("ERROR: Phase Resistance Measurement Timeout\n");
            return false;
        }

        float I_alpha = state_.I_a;
        test_voltage += (kI * sample_time_) * (test_current - I_alpha);

        if (test_voltage > max_voltage)
            test_voltage = max_voltage; // Clamp Current
        if (test_voltage < -max_voltage)
            test_voltage = -max_voltage; // Clamp Current

        // Test voltage along phase A
        controller->SetModulationOutput(0.0f, test_voltage, 0.0f);
    }

    float R = test_voltage / test_current;
    measurement.f32 = R;
    if (fabs(test_voltage) == fabs(max_voltage) || R < 0.01f || R > 1.0f)
    {
        //motor->error = ERROR_PHASE_RESISTANCE_OUT_OF_RANGE;
        Logger::Instance().Print("ERROR: Resistance Measurement Out of Range: %f\n\r", R);

        // Update Serial Interface
        CommandHandler::SendMeasurementComplete(command_feedback_t::MEASURE_RESISTANCE_COMPLETE, error_type_t::MEASUREMENT_OUT_OF_RANGE, measurement);
        config_.phase_resistance = R;
        return false;
    }
    config_.phase_resistance = R;
    Logger::Instance().Print("Phase Resistance: %f ohms\n", config_.phase_resistance);

    // Shutdown the phases
    controller->SetDuty(0.5f, 0.5f, 0.5f);

    osDelay(200);

    // Send Complete Feedback
    CommandHandler::SendMeasurementComplete(command_feedback_t::MEASURE_RESISTANCE_COMPLETE, error_type_t::SUCCESSFUL, measurement);
    return true;
}

bool Motor::OrderPhases(MotorController *controller)
{
    Logger::Instance().Print("[MOTOR]: \n\rRunning phase direction scan...\n\r");

    float theta_start = 0;
    float theta_end = 0;
    float test_voltage = config_.calib_current * config_.phase_resistance;
    float rotor_lock_duration = 2.0f;
    float scan_step_size = 1.0f / 5000.0f; // Amount to step in open loop
    float scan_range = 4.0f * M_PI;        // Scan range for phase order (electrical range)

    config_.phase_order = 1;               // Reset Phase Order to Default

    // Feedback measurement
    measurement_t measurement;
    measurement.i32 = config_.phase_order;

    //printf("Locking Rotor to D-Axis...\n\r");
    LockRotor(controller, rotor_lock_duration, test_voltage);
    //printf("Rotor stabilized.\n\r");

    Update(); // Update State/Position Sensor

    osDelay(1); // Wait a ms

    theta_start = state_.theta_mech_true;
    for (float ref_angle = 0; ref_angle < scan_range; ref_angle += scan_step_size)
    {
        if (osThreadFlagsWait(CURRENT_MEASUREMENT_COMPLETE_SIGNAL, osFlagsWaitAny, CURRENT_MEASUREMENT_TIMEOUT) != osErrorTimeout)
        {
            // TODO: Error
            Logger::Instance().Print("[MOTOR]: \n\rPhase Direction Scan Timeout!\n\r");
            // Send Complete Feedback
            CommandHandler::SendMeasurementComplete(command_feedback_t::MEASURE_PHASE_ORDER_COMPLETE, error_type_t::MEASUREMENT_TIMEOUT, measurement);
            return false;
        }
        // Set PWM Output
        controller->SetModulationOutput(ref_angle, test_voltage, 0.0f);

        Update(); // Update State/Position Sensor
    }
    theta_end = state_.theta_mech_true;

    //printf("Angle Start: %f, Angle End: %f\n\r", theta_start, theta_end);
    if (theta_end - theta_start > 0)
    {
        Logger::Instance().Print("[MOTOR]: \n\rPhase Order CORRECT.\n\r");
        //printf("Phase Order is CORRECT!\n\r");
        //rotor_sensor_->SetDirection(1);
        config_.phase_order = 1;
    }
    else
    {
        printf("Phase Order is INCORRECT!\n\r");
        Logger::Instance().Print("[MOTOR]: \n\rPhase Order REVERSED.\n\r");
        //rotor_sensor_->SetDirection(-1);
        config_.phase_order = 0;
    }
    measurement.i32 = config_.phase_order;
    // Send Complete Feedback
    CommandHandler::SendMeasurementComplete(command_feedback_t::MEASURE_PHASE_ORDER_COMPLETE, error_type_t::SUCCESSFUL, measurement);
    return true;
}

bool Motor::CalibrateEncoderOffset(MotorController *controller)
{
    Logger::Instance().Print("[MOTOR] Running Encoder Offset/Eccentricity Calibration......\n");


    measurement_t elec_offset;
    elec_offset.f32 = 0.0f;

    float rotor_lock_duration = 2.0f; // Rotor Lock Settling Time
    float *error_forward;  // Error Vector Forward Rotation
    float *error_backward; // Error Vector Backward Rotation
    int8_t *lookup_table; // Lookup Table
    int32_t *raw_forward;
    int32_t *raw_backward;
    float *error;
    float *error_filtered;

    const int32_t window = 128;
    const int32_t num_samples = 128 * config_.num_pole_pairs;                    // Num samples per mechanical rotation.  Multiple of NPP for filtering reasons (see later)
    const int32_t sub_samples = 40*4;                                              // increments between saved samples (for smoothing motion)
    float delta = 2 * PI * config_.num_pole_pairs / (num_samples * sub_samples); // change in angle between samples

    error_forward = new float[num_samples];
    error_backward = new float[num_samples];

    error = new float[num_samples];
    error_filtered = new float[num_samples];

    // Zero Array.  Do this explicitly in case compilers vary
    memset(error_forward, 0, sizeof(float)*num_samples);
    memset(error_backward, 0, sizeof(float)*num_samples);
    memset(error, 0, sizeof(float)*num_samples);
    memset(error_filtered, 0, sizeof(float)*num_samples);
    
    //float cogging_current[window] = {0};

    const int32_t num_lookups = 128;
    lookup_table = new int8_t[num_lookups]; // Clear the previous lookup table.

    // Zero Array.  Do this explicitly in case compilers vary
    memset(lookup_table, 0, sizeof(int8_t)*num_lookups);
    rotor_sensor_->Reset();
    
    //rotor_sensor_->SetOffsetLUT(lookup_table);
    //rotor_sensor_->SetElectricalOffset(0); // Clear Offset

    raw_forward = new int32_t[num_samples];
    raw_backward = new int32_t[num_samples];

    // Zero Array.  Do this explicitly in case compilers vary
    memset(raw_forward, 0, sizeof(int32_t)*num_samples);
    memset(raw_backward, 0, sizeof(int32_t)*num_samples);
    

    float theta_ref = 0;
    float theta_actual = 0;
    float test_voltage = config_.calib_current * config_.phase_resistance;

    //printf("Locking Rotor to D-Axis...\n\r");

    LockRotor(controller, rotor_lock_duration, test_voltage);

    //printf("Rotor stabilized.\n\r");
    Update();   // Update State/Position Sensor
    osDelay(1); // Wait a ms

    // TODO: Cogging Current

    //printf("\n\rCalibrating Forwards Direction...\n\r");
    Logger::Instance().Print("[MOTOR] Calibrating Forwards Direction...\n");
    // Rotate Forward
    for (int32_t i = 0; i < num_samples; i++)
    {
        for (int32_t j = 0; j < sub_samples; j++)
        {
            if (osThreadFlagsWait(CURRENT_MEASUREMENT_COMPLETE_SIGNAL, osFlagsWaitAny, CURRENT_MEASUREMENT_TIMEOUT) != osErrorTimeout) {
                // Send Complete Feedback
                CommandHandler::SendMeasurementComplete(command_feedback_t::MEASURE_ENCODER_OFFSET_COMPLETE, error_type_t::MEASUREMENT_TIMEOUT, elec_offset);
                return false;
            }
            theta_ref += delta;
            controller->SetModulationOutput(theta_ref, test_voltage, 0.0f);

            //wait_us(100); // Wait a bit.

            Update(); // Update State/Position Sensor
        }
        Update(); // Update State/Position Sensor

        theta_actual = rotor_sensor_->GetMechanicalPositionTrue(); // Get Mechanical Position
        error_forward[i] = theta_ref / config_.num_pole_pairs - theta_actual;
        raw_forward[i] = rotor_sensor_->GetRawPosition();
        //printf("%.4f   %.4f    %ld   %.4f to %.4f\n\r", theta_ref / (config_.num_pole_pairs), theta_actual, raw_forward[i], error_forward[i],theta_actual);
        //theta_ref += delta;
    }

    // Clear output
    //controller->SetModulationOutput(theta_ref, 0.0f, 0.0f);


    //printf("\r\nCooling Down 5s...\r\n");
    //wait(5); // 5 Seconds.  Let Motor Cool a bit since we are running open loop.  Can get warm.
    //printf("\n\rCalibrating Backwards Direction...\n\r");

    Logger::Instance().Print("[MOTOR] Calibrating Backwards Direction...\n");

    // Rotate Backwards
    for (int32_t i = 0; i < num_samples; i++)
    {
        for (int32_t j = 0; j < sub_samples; j++)
        {
            if (osThreadFlagsWait(CURRENT_MEASUREMENT_COMPLETE_SIGNAL, osFlagsWaitAny, CURRENT_MEASUREMENT_TIMEOUT) != osErrorTimeout) {
                // Send Complete Feedback
                CommandHandler::SendMeasurementComplete(command_feedback_t::MEASURE_ENCODER_OFFSET_COMPLETE, error_type_t::MEASUREMENT_TIMEOUT, elec_offset);
                 return false;
            }
            theta_ref -= delta;
            controller->SetModulationOutput(theta_ref, test_voltage, 0.0f);

            //wait_us(100);

            Update(); // Update State/Position Sensor
        }
        Update(); // Update State/Position Sensor

        theta_actual = rotor_sensor_->GetMechanicalPositionTrue(); // Get Mechanical Position
        error_backward[i] = theta_ref / config_.num_pole_pairs - theta_actual;
        raw_backward[i] = rotor_sensor_->GetRawPosition();
        //printf("%.4f   %.4f    %ld\n\r", theta_ref / (config_.num_pole_pairs), theta_actual, raw_backward[i]);
        //theta_ref -= delta;
        //printf(".\r");
    }

    // Compute Electrical Offset
    float offset = 0;
    for (int32_t i = 0; i < num_samples; i++)
    {
        offset += (error_forward[i] + error_backward[num_samples - 1 - i]) / (2.0f * num_samples); // calclate average position sensor offset
    }
    offset = fmod(offset * config_.num_pole_pairs, 2 * PI); // convert mechanical angle to electrical angle
    
    while(offset < 0) // Keep offset 0 to 2*PI
        offset += 2 * PI;

    rotor_sensor_->SetElectricalOffset(offset); // Set Offset
    elec_offset.f32 = offset;

    // Clear output
    controller->SetModulationOutput(theta_ref, 0.0f, 0.0f);

    // Perform filtering to linearize position sensor eccentricity
    // FIR n-sample average, where n = number of samples in one electrical cycle
    // This filter has zero gain at electrical frequency and all integer multiples
    // So cogging effects should be completely filtered out.

    float mean = 0;

    // Average Forward and Backward Directions
    for (int32_t i = 0; i < num_samples; i++)
    {
        error[i] = 0.5f * (error_forward[i] + error_backward[num_samples - i - 1]);
    }

    for (int32_t i = 0; i < num_samples; i++)
    {
        for (int32_t j = 0; j < window; j++)
        {
            int32_t index = -window / 2 + j + i; // Indices from -window/2 to + window/2
            if (index < 0)
            {
                index += num_samples;
            } // Moving average wraps around
            else if (index > num_samples - 1)
            {
                index -= num_samples;
            }
            //printf("ERROR: %d\r\n", error[index]);
            error_filtered[i] += error[index] / (float)window;
        }
        // if (i < window)
        // {
        //     cogging_current[i] = current * sinf((error[i] - error_filtered[i]) * config_.num_pole_pairs);
        // }
        //printf("%.4f   %4f    %.4f   %.4f\n\r", error[i], error_filtered[i], error_forward[i], error_backward[i]);
        mean += error_filtered[i] / num_samples;
    }

    int32_t raw_offset = (raw_forward[0] + raw_backward[num_samples - 1]) / 2; //Insensitive to errors in this direction, so 2 points is plenty

    
    //Logger::Instance().Print("\n\r Encoder non-linearity compensation table\n\r");
    //Logger::Instance().Print(" Sample Number : Lookup Index : Lookup Value\n\r\n\r");
    for (int32_t i = 0; i < num_lookups; i++) // Build Lookup Table
    {
        int32_t index = (raw_offset >> 7) + i;
        if (index > (num_lookups - 1))
        {
            index -= num_lookups;
        }
        lookup_table[index] = (int8_t)((error_filtered[i * config_.num_pole_pairs] - mean) * (float)(rotor_sensor_->GetCPR()) / (2.0f * PI));
        //Logger::Instance().Print("%ld   %ld   %ld\n\r", i, index, lookup_table[index]);
        //printf("%ld, %ld \n\r", i, lookup_table[index]);
        delay_us(20);
    }

    // TODO: Not quite working. Need to fix.  For now don't compensate eccentricity
    rotor_sensor_->SetOffsetLUT(lookup_table); // Write Compensated Lookup Table

    //memcpy(controller->cogging, cogging_current, sizeof(controller->cogging));  //compensation doesn't actually work yet....
    //printf("\n\rEncoder Electrical Offset (rad) %f\n\r", offset);
    Logger::Instance().Print("[MOTOR] Encoder Electrical Offset (rad) %f\n\r", offset);
    // Clear Memory
    delete[] error_forward; 
    delete[] error_backward;
    delete[] lookup_table;
    delete[] raw_forward;
    delete[] raw_backward;
    delete[] error;
    delete[] error_filtered;

    // Shutdown the phases
    controller->SetDuty(0.5f, 0.5f, 0.5f);

    // Send Complete Feedback
    CommandHandler::SendMeasurementComplete(command_feedback_t::MEASURE_ENCODER_OFFSET_COMPLETE, error_type_t::SUCCESSFUL, elec_offset);
    return true;
}

bool Motor::LockRotor(MotorController *controller, float lock_duration, float lock_voltage)
{
    // Lock rotor to zero phase, A/D-Axis
    for (int i = 0; i < lock_duration * (float)sample_time_; ++i)
    {
        if (osThreadFlagsWait(CURRENT_MEASUREMENT_COMPLETE_SIGNAL, osFlagsWaitAny, CURRENT_MEASUREMENT_TIMEOUT) != osErrorTimeout)
        {
            return false;
        }
        controller->SetModulationOutput(0.0f, lock_voltage, 0.0f);
    }
    return true;
}
void Motor::SetPolePairs(uint32_t pole_pairs)
{
    config_.num_pole_pairs = pole_pairs;

    // Compute other parameters
    config_.flux_linkage = 60.0f / (SQRT3 * config_.K_v * PI * config_.num_pole_pairs * 2);
    config_.K_t = config_.flux_linkage * config_.num_pole_pairs * 1.5f; // rotor_flux_*Pole_Pairs*3/2
    config_.K_t_out = config_.K_t * config_.gear_ratio;
    // Update Rotor
    rotor_sensor_->SetPolePairs(pole_pairs);
    dirty_ = true;
}

void Motor::SetKV(float K_v)
{
    config_.K_v = K_v;

    // Compute other parameters
    config_.flux_linkage = 60.0f / (SQRT3 * config_.K_v * PI * config_.num_pole_pairs * 2);
    config_.K_t = config_.flux_linkage * config_.num_pole_pairs * 1.5f; // rotor_flux_*Pole_Pairs*3/2
    config_.K_t_out = config_.K_t * config_.gear_ratio;

    dirty_ = true;
}
