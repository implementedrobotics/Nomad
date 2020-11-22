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
#include <RegisterInterface.h>
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
    config_.phase_resistance = 0.068850f;
    config_.phase_inductance_d = 0.000052f;
    config_.phase_inductance_q = 0.000052f;
    config_.phase_order = 1;
    config_.calib_current = 10.0f;
    config_.calib_voltage = 2.0f;
    config_.gear_ratio = 1.0f; // No Gearbox by default
    config_.calibrated = 1;

    // Update KV Calulations
    SetKV(K_v);


    // Setup Registers
    RegisterInterface::AddRegister(MotorConfigRegisters_e::MotorConfigRegister1, new Register((MotorConfigRegister1_t *)&config_, true));
    RegisterInterface::AddRegister(MotorConfigRegisters_e::PolePairs, new Register(&config_.num_pole_pairs));
    RegisterInterface::AddRegister(MotorConfigRegisters_e::K_v, new Register(&config_.K_v));
    RegisterInterface::AddRegister(MotorConfigRegisters_e::K_t, new Register(&config_.K_t));
    RegisterInterface::AddRegister(MotorConfigRegisters_e::K_t_out, new Register(&config_.K_t_out));
    RegisterInterface::AddRegister(MotorConfigRegisters_e::FluxLinkage, new Register(&config_.flux_linkage));
    RegisterInterface::AddRegister(MotorConfigRegisters_e::PhaseResistance, new Register(&config_.phase_resistance));
    RegisterInterface::AddRegister(MotorConfigRegisters_e::PhaseInductanceD, new Register(&config_.phase_inductance_d));
    RegisterInterface::AddRegister(MotorConfigRegisters_e::PhaseInductanceQ, new Register(&config_.phase_inductance_q));
    RegisterInterface::AddRegister(MotorConfigRegisters_e::GearRatio, new Register(&config_.gear_ratio));
    RegisterInterface::AddRegister(MotorConfigRegisters_e::PhaseOrder, new Register(&config_.phase_order));

    RegisterInterface::AddRegister(MotorConfigRegisters_e::MotorThermalConfigRegister, new Register((MotorThermalConfigRegister_t *)&config_.continuous_current_max, true));
    RegisterInterface::AddRegister(MotorConfigRegisters_e::ContinuousCurrentLimit, new Register(&config_.continuous_current_max));
    RegisterInterface::AddRegister(MotorConfigRegisters_e::ContinuousCurrentTau, new Register(&config_.continuous_current_tau));
    
    RegisterInterface::AddRegister(MotorConfigRegisters_e::MotorCalibrationConfigRegister, new Register((MotorCalibrationConfigRegister_t *)&config_.calib_current, true));
    RegisterInterface::AddRegister(MotorConfigRegisters_e::CalibrationCurrent, new Register(&config_.calib_current));
    RegisterInterface::AddRegister(MotorConfigRegisters_e::CalibrationVoltage, new Register(&config_.calib_voltage));

    RegisterInterface::AddRegister(MotorStateRegisters_e::MotorStateRegister1, new Register((MotorStateRegister1_t *)&state_, true));
    RegisterInterface::AddRegister(MotorStateRegisters_e::I_A, new Register(&state_.I_a));
    RegisterInterface::AddRegister(MotorStateRegisters_e::I_B, new Register(&state_.I_b));
    RegisterInterface::AddRegister(MotorStateRegisters_e::I_C, new Register(&state_.I_c));
    RegisterInterface::AddRegister(MotorStateRegisters_e::WindingsTemp, new Register(&state_.windings_temp));
    RegisterInterface::AddRegister(MotorStateRegisters_e::OutputPosition, new Register(&state_.theta_mech));
    RegisterInterface::AddRegister(MotorStateRegisters_e::OutputPositionTrue, new Register(&state_.theta_mech_true));
    RegisterInterface::AddRegister(MotorStateRegisters_e::OutputVelocity, new Register(&state_.theta_mech_dot));
    RegisterInterface::AddRegister(MotorStateRegisters_e::ElectricalPosition, new Register(&state_.theta_elec));
    RegisterInterface::AddRegister(MotorStateRegisters_e::ElectricalVelocity, new Register(&state_.theta_elec_dot));

    // Setup Position Sensor
    rotor_sensor_ = new PositionSensorAS5x47(sample_time_, config_.num_pole_pairs);
}

void Motor::PrintConfig()
{
     // Print Configs
    Logger::Instance().Print("Motor Config: PP: %d, I_max: %f, I_tau: %f, R: %f, L_d: %f, L_q: %f\r\n", config_.num_pole_pairs, config_.continuous_current_max, config_.continuous_current_tau, config_.phase_resistance, config_.phase_inductance_d, config_.phase_inductance_q);
    Logger::Instance().Print("Motor Config: K_v: %f, flux_linkage: %f, K_t: %f, K_t_out: %f, Gear: %f\r\n", config_.K_v, config_.flux_linkage, config_.K_t, config_.K_t_out, config_.gear_ratio);
    Logger::Instance().Print("Motor Config: Phase Order: %d, Calib I: %f, Calib V: %f, Calibrated: %d\r\n", config_.phase_order, config_.calib_current, config_.calib_voltage, config_.calibrated);
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
    config_.flux_linkage = 60.0f / (Core::Math::kSqrt3 * config_.K_v * M_PI * config_.num_pole_pairs * 2);
    config_.K_t = config_.flux_linkage * config_.num_pole_pairs * 1.5f;
    config_.K_t_out = config_.K_t * config_.gear_ratio;
    // Update Rotor
    rotor_sensor_->SetPolePairs(pole_pairs);
    dirty_ = true;
}

void Motor::SetKV(float K_v)
{
    config_.K_v = K_v;

    // Compute other parameters
    config_.flux_linkage = 60.0f / (Core::Math::kSqrt3 * config_.K_v * M_PI * config_.num_pole_pairs * 2);
    config_.K_t = config_.flux_linkage * config_.num_pole_pairs * 1.5f;
    config_.K_t_out = config_.K_t * config_.gear_ratio;

    dirty_ = true;
}
