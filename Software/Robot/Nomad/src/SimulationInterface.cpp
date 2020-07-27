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

                // Create Ports
                input_port_map_[InputPort::JOINT_CONTROL] = std::make_shared<Realtime::Port>("JOINT_CONTROL", Realtime::Port::Direction::INPUT, Realtime::Port::DataType::DOUBLE, 12, rt_period_);

                // Referenence Input Port
                input_port_map_[InputPort::IMU_READ] = std::make_shared<Realtime::Port>("IMU_READ", Realtime::Port::Direction::INPUT, Realtime::Port::DataType::DOUBLE, 10, rt_period_);
                input_port_map_[InputPort::JOINT_STATE_READ] = std::make_shared<Realtime::Port>("JOINT_STATE_READ", Realtime::Port::Direction::INPUT, Realtime::Port::DataType::DOUBLE, 36, rt_period_);

                // Outputs
                output_port_map_[OutputPort::IMU_STATE] = std::make_shared<Realtime::Port>("IMU_STATE", Realtime::Port::Direction::OUTPUT, Realtime::Port::DataType::BYTE, 1, rt_period_);
                output_port_map_[OutputPort::JOINT_STATE] = std::make_shared<Realtime::Port>("JOINT_STATE", Realtime::Port::Direction::OUTPUT, Realtime::Port::DataType::BYTE, 1, rt_period_);
            }

            void SimulationInterface::Run()
            {
                // // Read Command
                // bool success = GetInputPort(InputPort::IMU_READ)->Receive(imu_read_msg_);
                // if (success)
                // {
                //     std::cout << "GOT: " << imu_read_msg_.data[6] << std::endl;

                //     // Translate to output messages
                //     imu_state_.orientation[0] = imu_read_msg_.data[0];
                //     imu_state_.orientation[1] = imu_read_msg_.data[1];
                //     imu_state_.orientation[2] = imu_read_msg_.data[2];
                //     imu_state_.orientation[3] = imu_read_msg_.data[3];

                //     memcpy(imu_state_msg_.data.data(), &imu_state_, sizeof(imu_state_t));
                // }
                // else
                // {
                //     std::cout << "NO DATA!!!!" << std::endl;
                // }

                // success = GetInputPort(InputPort::JOINT_STATE_READ)->Receive(joint_state_read_msg_);
                // if (success)
                // {
                //     std::cout << "GOT: " << joint_state_read_msg_.data[0] << std::endl;
                // }
                // else
                // {
                //     std::cout << "NO DATA!!!!" << std::endl;
                // }
                // std::cout << imu_state_msg_.length << std::endl;

                // Publish
                bool send_status = GetOutputPort(OutputPort::IMU_STATE)->Send(imu_state_msg_);
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
