/*
 * StandController.cpp
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
#include <Nomad/Interface/StandController.hpp>

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
        namespace Interface
        {

            StandController::StandController(const std::string &name,
                                             const long rt_period,
                                             unsigned int rt_priority,
                                             const int rt_core_id,
                                             const unsigned int stack_size) : Realtime::RealTimeTaskNode(name, rt_period, rt_priority, rt_core_id, stack_size)
            {
                // Create Ports
                // Sim Inputs
                input_port_map_[InputPort::SIM_DATA] = Communications::Port<sim_data_t>::CreateInput("SIM_DATA", rt_period_);

                // Outputs
                output_port_map_[OutputPort::JOINT_CONTROL_CMD_OUT] = Communications::Port<joint_control_cmd_t>::CreateOutput("JOINT_CONTROL", rt_period_);
            }

            void StandController::Run()
            {
                //Systems::Time t;
                static uint64_t last_control_time = 0;
                static int lost_control = 0;

                // Read Plant State
                if(!GetInputPort(InputPort::SIM_DATA)->Receive(sim_data_, std::chrono::milliseconds(1000)))
                {
                    std::cout << "FAILED TO GET SYNCED SIM MESSAGE!!: " << lost_control++ << std::endl;
                    // TODO: What to do.
                }


                //std::cout << "Command In Time: " << Systems::Time::GetTimeStamp() << std::endl;
                //std::cout << "Stamps: " << imu_data_.sequence_num << " " << joint_state_.sequence_num << " " << com_state_.sequence_num << std::endl;

                //std::cout << "----------------------------Stamps: " << imu_data_.sequence_num << " " << joint_state_.sequence_num << " " << com_state_.sequence_num << std::endl;
                //std::cout << "Stamps: " << last_stamp_imu << " " << last_stamp_joint << " " << last_stamp_com << std::endl;
                //std::cout << "Stamps: " << imu_data_.sequence_num << " " << last_stamp << std::endl;

                //last_control_time = Systems::Time::GetTimeStamp();

                // Publish/Forward to Sim
                memset(&joint_command_, 0, sizeof(joint_control_cmd_t));
                {
                    //Systems::Time t;
                    GetOutputPort(OutputPort::JOINT_CONTROL_CMD_OUT)->Send(joint_command_);
                   // std::cout << "OUT SEND: " << joint_command_.sequence_num << std::endl;
                }


                // while (!GetInputPort(InputPort::COM_STATE_IN)->Receive(com_state_))
                // {
                // //std::cout << "Got 3: " << com_state_.pos[2] << std::endl;

                //     if (timeout++ > 500)
                //         break;
                // }

                //std::cout << "Command Out Time: " << joint_command_.timestamp << std::endl;
                //std::cout << "Turn around time: " << joint_command_.timestamp - last_control_time << std::endl;
                //std::cout << "OUT: " << joint_command_.sequence_num << " " << std::endl;

                //std::cout << "Total Time: " << Systems::Time::GetTimeStamp() - last_control_time << "us" << std::endl;
                //std::cout << "Loop Frequency: " << 1000000.0 / (double)(Systems::Time::GetTimeStamp() - last_control_time) << std::endl;
                last_control_time = Systems::Time::GetTimeStamp();

                
            }

            void StandController::Setup()
            {
                bool outputs_bound = true;
                for (int i = 0; i < OutputPort::NUM_OUTPUTS; i++) // Bind all of our output ports
                {
                    if (!GetOutputPort(i)->Bind())
                    {
                        outputs_bound = false;
                    }
                }

                std::cout << "[StandController]: "
                          << "StandController Interface Publisher Running!: " << outputs_bound << " " << std::endl;
            }

        } // namespace Interface
    }     // namespace Nomad
} // namespace Robot
