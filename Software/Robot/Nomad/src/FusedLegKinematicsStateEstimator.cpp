/*
 * FusedLegKinematicsStateEstimator.cpp
 *
 *  Created on: July 28, 2020
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
 */

// Primary Include
#include <Nomad/Estimators/FusedLegKinematicsStateEstimator.hpp>

// C System Includes

// C++ System Includes
#include <iostream>
#include <string>
#include <sstream>
#include <memory>

// Third-Party Includes
#include <zcm/zcm-cpp.hpp>

// Project Includes
#include <Realtime/RealTimeTask.hpp>
#include <Common/Time.hpp>

namespace Robot::Nomad::Estimators
{
    // TODO: Static Variable in "Physics" Class somewhere
    double kGravity = 9.81;

    FusedLegKinematicsStateEstimator::FusedLegKinematicsStateEstimator(const double T_s)  : SystemBlock("Leg_Kin_State_Estimator_Task", T_s)
    {
        // State Estimate Input Port
        input_port_map_[InputPort::IMU_DATA] = Communications::Port::CreateInput<imu_data_t>("IMU_STATE");
        input_port_map_[InputPort::JOINT_STATE] = Communications::Port::CreateInput<joint_state_t>("JOINT_STATE");
        input_port_map_[InputPort::COM_STATE] = Communications::Port::CreateInput<com_state_t>("POSE_STATE");
        input_port_map_[InputPort::FOOT_STATE] = Communications::Port::CreateInput<full_state_t>("FOOT_STATE");

        // State Estimate Output Port
        output_port_map_[OutputPort::BODY_STATE_HAT] = Communications::Port::CreateOutput("BODY_STATE_HAT");
        output_port_map_[OutputPort::BODY_STATE_ACTUAL] = Communications::Port::CreateOutput("BODY_STATE_ACTUAL");
    }

        // Update function for stateful outputs
    void FusedLegKinematicsStateEstimator::UpdateStateOutputs()
    {
        // Receive Data
    }

    // Update function for stateless outputs
    void FusedLegKinematicsStateEstimator::UpdateStatelessOutputs()
    {
        // Read Inputs
        GetInputPort(InputPort::COM_STATE)->Receive(com_state_in_);

        com_state_out_.orientation[0] = com_state_in_.orientation[0];
        com_state_out_.orientation[1] = com_state_in_.orientation[1];
        com_state_out_.orientation[2] = com_state_in_.orientation[2];
        com_state_out_.orientation[3] = com_state_in_.orientation[3];

        com_state_out_.theta[0] = com_state_in_.theta[0];
        com_state_out_.theta[1] = com_state_in_.theta[1];
        com_state_out_.theta[2] = com_state_in_.theta[2];

        com_state_out_.pos[0] = com_state_in_.pos[0];
        com_state_out_.pos[1] = com_state_in_.pos[1];
        com_state_out_.pos[2] = com_state_in_.pos[2];

        com_state_out_.omega[0] = com_state_in_.omega[0];
        com_state_out_.omega[1] = com_state_in_.omega[1];
        com_state_out_.omega[2] = com_state_in_.omega[2];

        com_state_out_.vel[0] = com_state_in_.vel[0];
        com_state_out_.vel[1] = com_state_in_.vel[1];
        com_state_out_.vel[2] = com_state_in_.vel[2];

        // Compute State Estimate
        com_state_hat_.orientation[0] = com_state_in_.orientation[0];
        com_state_hat_.orientation[1] = com_state_in_.orientation[1];
        com_state_hat_.orientation[2] = com_state_in_.orientation[2];
        com_state_hat_.orientation[3] = com_state_in_.orientation[3];

        com_state_hat_.theta[0] = com_state_in_.theta[0];
        com_state_hat_.theta[1] = com_state_in_.theta[1];
        com_state_hat_.theta[2] = com_state_in_.theta[2];

        com_state_hat_.pos[0] = com_state_in_.pos[0];
        com_state_hat_.pos[1] = com_state_in_.pos[1];
        com_state_hat_.pos[2] = com_state_in_.pos[2];

        com_state_hat_.omega[0] = com_state_in_.omega[0];
        com_state_hat_.omega[1] = com_state_in_.omega[1];
        com_state_hat_.omega[2] = com_state_in_.omega[2];

        com_state_hat_.vel[0] = com_state_in_.vel[0];
        com_state_hat_.vel[1] = com_state_in_.vel[1];
        com_state_hat_.vel[2] = com_state_in_.vel[2];

        GetOutputPort(OutputPort::BODY_STATE_HAT)->Send(com_state_hat_);

    }

    // Update function for next state from inputs
    void FusedLegKinematicsStateEstimator::UpdateState()
    {

    }


} // namespace Robot::Nomad::Estimators
