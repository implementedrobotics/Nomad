/*
 * SimulationInterface.cpp
 *
 *  Created on: July 16, 2020
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

// Primary Include
#include <Nomad/Interface/SimulationInterface.hpp>

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
#include <Communications/Messages/double_vec_t.hpp>
#include <Communications/Messages/generic_msg_t.hpp>
#include <Common/Time.hpp>

namespace Robot
{
    namespace Nomad
    {
        namespace Interface
        {

            SimulationInterface::SimulationInterface(const std::string &name,
                                                     const long rt_period,
                                                     unsigned int rt_priority,
                                                     const int rt_core_id,
                                                     const unsigned int stack_size) : Realtime::RealTimeTaskNode(name, rt_period, rt_priority, rt_core_id, stack_size)
            {

                // IMU Messages
                imu_state_msg_.length = sizeof(imu_state_t);
                imu_state_msg_.data.resize(imu_state_msg_.length);

                // Intialize to 'zero' state
                memset(&imu_state_, 0, sizeof(imu_state_t));

                // Joint State Messages
                joint_state_msg_.length = sizeof(joint_state_t);
                joint_state_msg_.data.resize(joint_state_msg_.length);

                // Intialize to 'zero' state
                memset(&joint_state_, 0, sizeof(joint_state_t));

                pose_ground_truth_msg_.length = 6;
                pose_ground_truth_msg_.data.resize(pose_ground_truth_msg_.length);
                memset(&joint_state_, 0, sizeof(joint_state_t));

                // Create Ports
                input_port_map_[InputPort::JOINT_CONTROL] = std::make_shared<Realtime::Port>("JOINT_CONTROL", Realtime::Port::Direction::INPUT, Realtime::Port::DataType::DOUBLE, 12, rt_period_);

                // Referenence Input Port
                input_port_map_[InputPort::IMU_READ] = std::make_shared<Realtime::Port>("IMU_READ", Realtime::Port::Direction::INPUT, Realtime::Port::DataType::DOUBLE, 10, rt_period_);
                input_port_map_[InputPort::JOINT_STATE_READ] = std::make_shared<Realtime::Port>("JOINT_STATE_READ", Realtime::Port::Direction::INPUT, Realtime::Port::DataType::DOUBLE, 36, rt_period_);
                input_port_map_[InputPort::CHEATER_POSE_READ] = std::make_shared<Realtime::Port>("CHEATER_POSE_READ", Realtime::Port::Direction::INPUT, Realtime::Port::DataType::DOUBLE, 6, rt_period_);

                // Outputs
                output_port_map_[OutputPort::IMU_STATE] = std::make_shared<Realtime::Port>("IMU_STATE", Realtime::Port::Direction::OUTPUT, Realtime::Port::DataType::BYTE, 1, rt_period_);
                output_port_map_[OutputPort::JOINT_STATE] = std::make_shared<Realtime::Port>("JOINT_STATE", Realtime::Port::Direction::OUTPUT, Realtime::Port::DataType::BYTE, 1, rt_period_);
                output_port_map_[OutputPort::GROUND_TRUTH] = std::make_shared<Realtime::Port>("CHEATER_POSE", Realtime::Port::Direction::OUTPUT, Realtime::Port::DataType::DOUBLE, 6, rt_period_);

            }

            void SimulationInterface::Run()
            {
                
                // Read Any Input Commands


                // Read Plant State
                if(GetInputPort(InputPort::IMU_READ)->Receive(imu_read_msg_))
                {
                    // Translate to output messages
                    imu_state_.orientation[0] = imu_read_msg_.data[0];
                    imu_state_.orientation[1] = imu_read_msg_.data[1];
                    imu_state_.orientation[2] = imu_read_msg_.data[2];
                    imu_state_.orientation[3] = imu_read_msg_.data[3];

                    imu_state_.accel[0] = imu_read_msg_.data[4];
                    imu_state_.accel[1] = imu_read_msg_.data[5];
                    imu_state_.accel[2] = imu_read_msg_.data[6];

                    imu_state_.omega[0] = imu_read_msg_.data[7];
                    imu_state_.omega[1] = imu_read_msg_.data[8];
                    imu_state_.omega[2] = imu_read_msg_.data[9];

                    memcpy(imu_state_msg_.data.data(), &imu_state_, sizeof(imu_state_t));
                }

                if (GetInputPort(InputPort::JOINT_STATE_READ)->Receive(joint_state_read_msg_))
                {
                    for(int i = 0; i < 12; i++)
                    {
                        joint_state_.q_hat[i] = joint_state_read_msg_.data[i];
                    }

                    for(int i = 0; i < 12; i++)
                    {
                        joint_state_.q_dot_hat[i] = joint_state_read_msg_.data[i+12];
                    }

                    for(int i = 0; i < 12; i++)
                    {
                        joint_state_.torque_hat[i] = joint_state_read_msg_.data[i+24];
                    }

                    memcpy(joint_state_msg_.data.data(), &joint_state_, sizeof(joint_state_t));
                }

                if (GetInputPort(InputPort::CHEATER_POSE_READ)->Receive(cheater_pose_read_msg_))
                {
                    pose_ground_truth_msg_.data[0] = cheater_pose_read_msg_.data[0];
                    pose_ground_truth_msg_.data[1] = cheater_pose_read_msg_.data[1];
                    pose_ground_truth_msg_.data[2] = cheater_pose_read_msg_.data[2];
                    pose_ground_truth_msg_.data[3] = cheater_pose_read_msg_.data[3];
                    pose_ground_truth_msg_.data[4] = cheater_pose_read_msg_.data[4];
                    pose_ground_truth_msg_.data[5] = cheater_pose_read_msg_.data[5];
                }

                // Publish Output
                GetOutputPort(OutputPort::IMU_STATE)->Send(imu_state_msg_);
                GetOutputPort(OutputPort::JOINT_STATE)->Send(joint_state_msg_);
                GetOutputPort(OutputPort::GROUND_TRUTH)->Send(pose_ground_truth_msg_);
            }

            void SimulationInterface::Setup()
            {
                // Connect Input Ports
                bool inputs_connected = true;
                for (int i = 0; i < NUM_INPUTS; i++) // Connect all of our input ports
                {
                    if (!GetInputPort(i)->Connect())
                    {
                        inputs_connected = false;
                    }
                }

                bool outputs_bound = true;
                for (int i = 0; i < NUM_OUTPUTS; i++) // Bind all of our output ports
                {
                    if (!GetOutputPort(i)->Bind())
                    {
                        outputs_bound = false;
                    }
                }

                std::cout << "[SimulationInterface]: "
                          << "Simulation Interface Publisher Running!: " << outputs_bound << " " << inputs_connected << std::endl;
            }

        } // namespace Interface
    }     // namespace Nomad
} // namespace Robot
