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

    FusedLegKinematicsStateEstimator::FusedLegKinematicsStateEstimator(const std::string &name,
                                                                       const long rt_period,
                                                                       unsigned int rt_priority,
                                                                       const int rt_core_id,
                                                                       const unsigned int stack_size) : Realtime::RealTimeTaskNode(name, rt_period, rt_priority, rt_core_id, stack_size)
    {
        // State Estimate Input Port
        input_port_map_[InputPort::IMU_DATA] = Communications::Port::CreateInput<imu_data_t>("IMU_STATE", rt_period_);
        input_port_map_[InputPort::JOINT_STATE] = Communications::Port::CreateInput<joint_state_t>("JOINT_STATE", rt_period_);
        input_port_map_[InputPort::COM_STATE] = Communications::Port::CreateInput<com_state_t>("POSE_STATE", rt_period_);
        input_port_map_[InputPort::FOOT_STATE] = Communications::Port::CreateInput<full_state_t>("FOOT_STATE", rt_period_);

        // State Estimate Output Port
        output_port_map_[OutputPort::BODY_STATE_HAT] = Communications::Port::CreateOutput("BODY_STATE_HAT", rt_period_);
        output_port_map_[OutputPort::BODY_STATE_ACTUAL] = Communications::Port::CreateOutput("BODY_STATE_ACTUAL", rt_period_);
    }

    void FusedLegKinematicsStateEstimator::Run()
    {
        // Estimate State
        // if (GetInputPort(InputPort::IMU_DATA)->Receive(imu_data_))
        // {
        // }

        // if (GetInputPort(InputPort::JOINT_STATE)->Receive(joint_state_))
        // {
        // }

        // if (GetInputPort(InputPort::FOOT_STATE)->Receive(full_state_in_))
        // {
        // }
        
        
        if (GetInputPort(InputPort::COM_STATE)->Receive(com_state_in_, std::chrono::microseconds(100000)))
        {
            //std::cout << "GOT COM STATE: " << com_state_in_.pos[2] << std::endl;
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
        }

        else
        {
            std::cout << "GOT NOTHING IN STATE ESTIMATE!" << std::endl;
        }
        

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

        // Publish State
        bool send_status = GetOutputPort(OutputPort::BODY_STATE_HAT)->Send(com_state_hat_);
        //send_status = GetOutputPort(OutputPort::BODY_STATE_ACTUAL)->Send(com_state_out_);

        //std::cout << "[FusedLegKinematicsStateEstimator]: Publishing: " << com_state_hat_.pos[2] << " Send: " << send_status << std::endl;
    }

    void FusedLegKinematicsStateEstimator::Setup()
    {
        bool outputs_bound = true;
        for (int i = 0; i < OutputPort::NUM_OUTPUTS; i++) // Bind all of our output ports
        {
            if (!GetOutputPort(i)->Bind())
            {
                outputs_bound = false;
            }
        }
        std::cout << "[FusedLegKinematicsStateEstimator]: "
                  << "State Estimator Publisher Running!: " << outputs_bound << std::endl;
    }
} // namespace Robot::Nomad::Estimators
