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

                // Create Ports
                // Sim Inputs
                input_port_map_[InputPort::IMU_STATE_IN] = Realtime::Port::CreateInput<imu_data_t>("IMU_STATE", rt_period_);
                input_port_map_[InputPort::JOINT_STATE_IN] = Realtime::Port::CreateInput<joint_state_t>("JOINT_STATE", rt_period_);
                input_port_map_[InputPort::COM_STATE_IN] = Realtime::Port::CreateInput<com_state_t>("POSE_STATE", rt_period_);

                // Control Inputs
                input_port_map_[InputPort::JOINT_CONTROL_CMD_IN] = Realtime::Port::CreateInput<joint_control_cmd_t>("JOINT_CONTROL", rt_period_);

                // Outputs
                output_port_map_[OutputPort::IMU_STATE_OUT] = Realtime::Port::CreateOutput("IMU_STATE", rt_period_);
                output_port_map_[OutputPort::JOINT_STATE_OUT] = Realtime::Port::CreateOutput("JOINT_STATE", rt_period_);
                output_port_map_[OutputPort::COM_STATE_OUT] = Realtime::Port::CreateOutput("POSE_STATE", rt_period_);
            }

            void SimulationInterface::Run()
            {

                // Read Any Input Commands

                //Read Plant State
                if (GetInputPort(InputPort::IMU_STATE_IN)->Receive(imu_data_))
                {
                    //std::cout << "Got: " << imu_data_.accel[2] << std::endl;
                }

                if (GetInputPort(InputPort::JOINT_STATE_IN)->Receive(joint_state_))
                {
                   // std::cout << "Got 2: " << joint_state_.q[2] << std::endl;
                }

                if (GetInputPort(InputPort::COM_STATE_IN)->Receive(com_state_))
                {
                   // std::cout << "Got 3: " << com_state_.pos[2] << std::endl;
                }

                //Publish/Forward Outputs from Sim
                GetOutputPort(OutputPort::IMU_STATE_OUT)->Send(imu_data_);
                GetOutputPort(OutputPort::JOINT_STATE_OUT)->Send(joint_state_);
                GetOutputPort(OutputPort::COM_STATE_OUT)->Send(com_state_);
            }

            void SimulationInterface::Setup()
            {
                bool outputs_bound = true;
                for (int i = 0; i < OutputPort::NUM_OUTPUTS; i++) // Bind all of our output ports
                {
                    if (!GetOutputPort(i)->Bind())
                    {
                        outputs_bound = false;
                    }
                }

                std::cout << "[SimulationInterface]: "
                          << "Simulation Interface Publisher Running!: " << outputs_bound << " " <<  std::endl;
            }

        } // namespace Interface
    }     // namespace Nomad
} // namespace Robot
