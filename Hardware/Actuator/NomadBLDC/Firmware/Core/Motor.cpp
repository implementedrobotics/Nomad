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
#include "mbed.h"
#include "MotorController.h"
#include "../../math_ops.h"

Motor::Motor(float sample_time, float K_v, uint32_t pole_pairs) : sample_time_(sample_time),
                                                                  dirty_(false)
{
    // Zero State
    memset(&state_, 0, sizeof(state_));

    // TODO: Check for FLASH, otherwise defaults

    // Setup Default Configs
    config_.num_pole_pairs = pole_pairs;
    config_.phase_resistance = 0.0f;
    config_.phase_inductance_d = 0.0f;
    config_.phase_inductance_q = 0.0f;
    config_.phase_order = 0;
    config_.calibrated = false;

    // Update KV Calulations
    SetKV(K_v);

    // Setup Position Sensor
    rotor_sensor_ = new PositionSensorAS5x47(sample_time_, config_.num_pole_pairs);
}

void Motor::Update()
{
    // Update Position Sensor
    rotor_sensor_->Update(sample_time_);

    // Update State
    state_.theta_mech = rotor_sensor_->GetMechanicalPosition();
    state_.theta_mech_dot = rotor_sensor_->GetMechanicalVelocity();
    state_.theta_elec = rotor_sensor_->GetElectricalPosition();
    state_.theta_elec_dot = rotor_sensor_->GetElectricalVelocity();

    // Update Temparature Observer
}
bool Motor::Calibrate(MotorController *controller)
{

    controller->SetDuty(0.5f, 0.5f, 0.5f); // Make sure we have no PWM period

    // TODO: Check Error Here
    // Measure Resistance
    MeasureMotorResistance(controller, 15.0f, 3.0f);

    // AND Here
    // Measure Inductance
    MeasureMotorInductance(controller, -2.0f, 2.0f);

    controller->SetDuty(0.5f, 0.5f, 0.5f); // Make sure we have no PWM period

    config_.calibrated = true; // Update Flag
    dirty_ = true;

    return true;
}

bool Motor::MeasureMotorInductance(MotorController *controller, float voltage_low, float voltage_high)
{
    printf("Measure Motor Inductance:\n\r");

    float test_voltages[2] = {voltage_low, voltage_high};
    float Ialphas[2] = {0.0f};
    float U = 0, V = 0, W = 0;
    float dtc_U = 0, dtc_V = 0, dtc_W = 0;

    static const int num_cycles = 1.5f / sample_time_; // Test runs for 3s;

    // Shutdown Phases
    controller->SetDuty(0.5f, 0.5f, 0.5f);

    for (int t = 0; t < num_cycles; ++t)
    {
        for (int i = 0; i < 2; ++i)
        {
            osEvent evt = osSignalWait(CURRENT_MEASUREMENT_COMPLETE_SIGNAL, CURRENT_MEASUREMENT_TIMEOUT);
            if (evt.status != osEventSignal)
            {
                // motor->error = ERROR_PHASE_INDUCTANCE_MEASUREMENT_TIMEOUT;
                printf("ERROR: Phase Inductance Measurement Timeout\n\r");
                return false;
            }
            if (i == 1) // When you step you are reading the previous step.  TODO: Make this Better!
                Ialphas[0] += state_.I_a;
            else
                Ialphas[1] += state_.I_a;

            // Test voltage along phase A
            controller->dqInverseTransform(0.0f, test_voltages[i], 0.0f, &U, &V, &W);
            controller->SVM(U, V, W, &dtc_U, &dtc_V, &dtc_W);
            controller->SetDuty(dtc_U, dtc_V, dtc_W);
            //SetVoltageTimings(test_voltages[i], 0.0f);
        }
    }

    // Shutdown phases
    controller->SetDuty(0.5f, 0.5f, 0.5f);
    //printf("I Alpha: %f/%f\n\r", Ialphas[0], Ialphas[1]);
    float v_L = 0.5f * (voltage_high - voltage_low);

    // Note: A more correct formula would also take into account that there is a finite timestep.
    // However, the discretisation in the current control loop inverts the same discrepancy
    float dI_by_dt = (Ialphas[1] - Ialphas[0]) / (sample_time_ * (float)num_cycles);
    float L = v_L / dI_by_dt;

    // TODO arbitrary values set for now
    if (L < 1e-6f || L > 500e-6f)
    {
        //motor->error = ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE;
        printf("ERROR: Inductance Measurement Out of Range: %f\n\r", L);
        return false;
    }

    // PMSM D/Q Inductance are the same.
    config_.phase_inductance_d = L;
    config_.phase_inductance_q = L;
    printf("Phase Inductance: %f Henries\r\n", config_.phase_inductance_d);

    return true;
}

bool Motor::MeasureMotorResistance(MotorController *controller, float test_current, float max_voltage)
{
    static const float kI = 10.0f;                          // [(V/s)/A]
    static const int num_test_cycles = 3.0f / sample_time_; // Test runs for 3s
    float test_voltage = 0.0f;
    float U = 0, V = 0, W = 0;
    float dtc_U = 0, dtc_V = 0, dtc_W = 0;
    //int sum = 0;
    printf("Measure Motor Resistance:\n\r");

    for (int i = 0; i < num_test_cycles; ++i)
    {
        osEvent evt = osSignalWait(CURRENT_MEASUREMENT_COMPLETE_SIGNAL, CURRENT_MEASUREMENT_TIMEOUT);
        if (evt.status != osEventSignal)
        {
            // motor->error = ERROR_PHASE_RESISTANCE_MEASUREMENT_TIMEOUT;
            printf("ERROR: Phase Resistance Measurement Timeout\n\r");
            return false;
        }

        float Ialpha = state_.I_a;
        test_voltage += (kI * sample_time_) * (test_current - Ialpha);

        if (test_voltage > max_voltage)
            test_voltage = max_voltage; // Clamp Current
        if (test_voltage < -max_voltage)
            test_voltage = -max_voltage; // Clamp Current

        // Test voltage along phase A
        controller->dqInverseTransform(0.0f, test_voltage, 0.0f, &U, &V, &W);
        controller->SVM(U, V, W, &dtc_U, &dtc_V, &dtc_W);
        controller->SetDuty(dtc_U, dtc_V, dtc_W);
        //controller->ParkInverseTransform(0.0f, test_voltage, 0.0f,  &v_alpha, &v_beta); // No need for a transform A axis = Alpha axis
        //controller->ClarkeInverseTransform(v_alpha, v_beta,
        //SetVoltageTimings(test_voltage, 0.0f);
    }

    float R = test_voltage / test_current;
    if (fabs(test_voltage) == fabs(max_voltage) || R < 0.01f || R > 1.0f)
    {
        // motor->error = ERROR_PHASE_RESISTANCE_OUT_OF_RANGE;
        printf("ERROR: Resistance Measurement Out of Range: %f\n\r", R);
        return false;
    }
    config_.phase_resistance = R;
    printf("Phase Resistance: %f ohms\r\n", config_.phase_resistance);

    // Shutdown the phases
    controller->SetDuty(0.5f, 0.5f, 0.5f);

    osDelay(200);
    return true;
}
void Motor::SetPolePairs(uint32_t pole_pairs)
{
    config_.num_pole_pairs = pole_pairs;
    dirty_ = true;
}

void Motor::SetKV(float K_v)
{
    config_.K_v = K_v;

    // Compute other parameters
    config_.flux_linkage = 60.0f / (SQRT3 * config_.K_v * PI * config_.num_pole_pairs * 2);
    config_.K_t = config_.flux_linkage * config_.num_pole_pairs * 1.5f; // rotor_flux_*Pole_Pairs*3/2

    dirty_ = true;
}
