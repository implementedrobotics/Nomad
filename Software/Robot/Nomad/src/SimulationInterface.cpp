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
                // Create Ports

                input_port_map_[InputPort::JOINT_CONTROL] = std::make_shared<Realtime::Port>("JOINT_CONTROL", Realtime::Port::Direction::INPUT, Realtime::Port::DataType::DOUBLE, 12, rt_period_);

                // Referenence Input Port
                input_port_map_[InputPort::IMU_READ] = std::make_shared<Realtime::Port>("IMU_READ", Realtime::Port::Direction::INPUT, Realtime::Port::DataType::DOUBLE, 10, rt_period_);

                //output_port_map_[OutputPort::FORCES] = std::make_shared<Realtime::Port>("FORCES", Realtime::Port::Direction::OUTPUT, Realtime::Port::DataType::DOUBLE, num_inputs_, rt_period_);
            }

            void SimulationInterface::Run()
            {
                // Read Command
                bool success = GetInputPort(InputPort::IMU_READ)->Receive(imu_read_msg_);
                if(success)
                {
                    std::cout << "GOT: " << imu_read_msg_.data[6] << std::endl;
                }
                else
                {
                    std::cout << "NO DATA!!!!" << std::endl;
                }
                
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
