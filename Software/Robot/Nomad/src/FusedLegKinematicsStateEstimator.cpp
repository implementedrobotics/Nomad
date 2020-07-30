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

namespace Robot
{
    namespace Nomad
    {
        namespace Estimators
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
                input_port_map_[InputPort::IMU_DATA] = Realtime::Port::CreateInput<imu_data_t>("IMU_STATE", rt_period_);
                input_port_map_[InputPort::JOINT_STATE] = Realtime::Port::CreateInput<joint_state_t>("JOINT_STATE", rt_period_);
                input_port_map_[InputPort::COM_STATE] = Realtime::Port::CreateInput<com_state_t>("POSE_STATE", rt_period_);

                // State Estimate Output Port
                output_port_map_[OutputPort::BODY_STATE_HAT] = Realtime::Port::CreateOutput("BODY_STATE_HAT", rt_period_);
            }

            void FusedLegKinematicsStateEstimator::Run()
            {
                // Estimate State
                if (GetInputPort(InputPort::IMU_DATA)->Receive(imu_data_))
                {
                }

                if (GetInputPort(InputPort::JOINT_STATE)->Receive(joint_state_))
                {
                }

                if (GetInputPort(InputPort::COM_STATE)->Receive(com_state_))
                {
                    //std::cout << "GOT COM STATE: " << com_state_.pos[2] << std::endl;
                }
                

                // Eigen::VectorXd x_hat_ = Eigen::Map<Eigen::VectorXd>(imu_data_in_.data.data(), num_states_);
                //std::cout << "[StateEstimator]: Received: " << x_hat_in_.sequence_num <<  std::endl;

                // Compute State Estimate
                // Eigen::VectorXd body_hat = Eigen::VectorXd::Ones(num_states_);

                // // Update State
                // com_state_out_.data[Idx::PHI] = body_hat[0];    // Roll Orientation
                // com_state_out_.data[Idx::THETA] = body_hat[1];  // Pitch Orientation
                // com_state_out_.data[Idx::PSI] = body_hat[2];    // Yaw Orientation
                // com_state_out_.data[Idx::X] = body_hat[3];      // X Position
                // com_state_out_.data[Idx::Y] = body_hat[4];      // Y Position
                // com_state_out_.data[Idx::Z] = body_hat[5];      // Z Position
                // com_state_out_.data[Idx::W_X] = body_hat[6];    // Roll Rate
                // com_state_out_.data[Idx::W_Y] = body_hat[7];    // Pitch Rate
                // com_state_out_.data[Idx::W_Z] = body_hat[8];    // Yaw Rate
                // com_state_out_.data[Idx::X_DOT] = body_hat[9];  // X Velocity
                // com_state_out_.data[Idx::Y_DOT] = body_hat[10]; // Y Velocity
                // com_state_out_.data[Idx::Z_DOT] = body_hat[11]; // Z Velocity

                // output_state_.data[Idx::GRAVITY] = x_hat_[12]; // Gravity

                //std::cout << "State Estimator Send: " << std::endl;

                // Publish State
                bool send_status = GetOutputPort(OutputPort::BODY_STATE_HAT)->Send(com_state_out_);

                //std::cout << "[FusedLegKinematicsStateEstimator]: Publishing: " << com_state_out_.data[Idx::X] << " Send: " << send_status << std::endl;
            }

            void FusedLegKinematicsStateEstimator::Setup()
            {

                GetOutputPort(OutputPort::BODY_STATE_HAT)->Bind();
                std::cout << "[FusedLegKinematicsStateEstimator]: "
                           << "State Estimator Publisher Running!: " << std::endl;
            }
        } // namespace Estimators
    }     // namespace Nomad
} // namespace Robot
