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
#include <TaskRead.hpp>

// C System Includes

// C++ System Includes
#include <iostream>
#include <string>
#include <sstream>
#include <memory>

// Third-Party Includes

// Project Includes
#include <Realtime/RealTimeTask.hpp>
#include <Common/Time.hpp>

using namespace std::chrono_literals;
namespace Robot
{
    namespace Nomad
    {
        namespace Interface
        {

            TaskRead::TaskRead(const std::string &name,
                                             const long rt_period,
                                             unsigned int rt_priority,
                                             const int rt_core_id,
                                             const unsigned int stack_size) : Realtime::RealTimeTaskNode(name, rt_period, rt_priority, rt_core_id, stack_size)
            {
                // Create Ports
                // Sim Inputs
                input_port_map_[InputPort::JOINT_CONTROL_CMD_IN] = Communications::Port::CreateInput<joint_control_cmd_t>("JOINT_CMD", rt_period_);
                //input_port_map_[InputPort::JOINT_STATE_IN] = Communications::Port::CreateInput<joint_state_t>("JOINT_STATE", rt_period_);
                //input_port_map_[InputPort::COM_STATE_IN] = Communications::Port::CreateInput<com_state_t>("POSE_STATE", rt_period_);

                // Outputs
                //output_port_map_[OutputPort::JOINT_CONTROL_CMD_OUT] = Communications::Port::CreateOutput("JOINT_CONTROL", rt_period_);

                memset(&joint_command_, 0, sizeof(joint_control_cmd_t));
            }

            void TaskRead::Run()
            {
                //Read Plant State
                // if (GetInputPort(InputPort::IMU_STATE_IN)->Receive(imu_data_))
                // {
                //     //std::cout << "Got: " << imu_data_.sequence_num << std::endl;
                // }

                // if (GetInputPort(InputPort::JOINT_STATE_IN)->Receive(joint_state_))
                // {
                // //    // std::cout << "Got 2: " << joint_state_.q[2] << std::endl;
                // }

                // if (GetInputPort(InputPort::COM_STATE_IN)->Receive(com_state_))
                // {
                // //    // std::cout << "Got 3: " << com_state_.pos[2] << std::endl;
                // }
               // std::cout << "RUNNING! " << std::endl;
                bool recv = false;
                {
                //Systems::Time t;
                recv = GetInputPort(InputPort::JOINT_CONTROL_CMD_IN)->Receive(joint_command_, 50us);
                }
                //std::cout << "REceived: " << joint_command_.sequence_num << " New: " << recv << std::endl;

                //std::cout << "Command In Time: " << Systems::Time::GetTimeStamp() << std::endl;
                //std::cout << "Stamps: " << imu_data_.sequence_num << " " << joint_state_.sequence_num << " " << com_state_.sequence_num << std::endl;

                //std::cout << "----------------------------Stamps: " << imu_data_.sequence_num << " " << joint_state_.sequence_num << " " << com_state_.sequence_num << std::endl;
                //std::cout << "Stamps: " << last_stamp_imu << " " << last_stamp_joint << " " << last_stamp_com << std::endl;
                //std::cout << "Stamps: " << imu_data_.sequence_num << " " << last_stamp << std::endl;

                //last_control_time = Systems::Time::GetTimeStamp();

                //GetOutputPort(OutputPort::JOINT_CONTROL_CMD_OUT)->Send(joint_command_);

            }

            void TaskRead::Setup()
            {
                bool outputs_bound = true;
                for (int i = 0; i < OutputPort::NUM_OUTPUTS; i++) // Bind all of our output ports
                {
                    if (!GetOutputPort(i)->Bind())
                    {
                        outputs_bound = false;
                    }
                }

                std::cout << "[TaskRead]: "
                          << "TaskRead Interface Publisher Running!: " << outputs_bound << " " << std::endl;
            }

        } // namespace Interface
    }     // namespace Nomad
} // namespace Robot
