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
#include "motor_controller_interface.h"
#include "Logger.h"
#include <Utilities/math.h>
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
    config_.calib_current = 10.0f;
    config_.calib_voltage = 2.0f;
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

void Motor::ZeroOutputPosition()
{
    Update(); // Make sure we are updated
    rotor_sensor_->ZeroPosition();
    Update(); // Post update
}



void Motor::SetPolePairs(uint32_t pole_pairs)
{
    config_.num_pole_pairs = pole_pairs;
    
    // Compute other parameters
    config_.flux_linkage = 60.0f / (Core::Math::kSqrt3 * config_.K_v * PI * config_.num_pole_pairs * 2);
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
    config_.flux_linkage = 60.0f / (Core::Math::kSqrt3 * config_.K_v * PI * config_.num_pole_pairs * 2);
    config_.K_t = config_.flux_linkage * config_.num_pole_pairs * 1.5f; // rotor_flux_*Pole_Pairs*3/2
    config_.K_t_out = config_.K_t * config_.gear_ratio;

    dirty_ = true;
}
