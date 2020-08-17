// /*
//  * SimulationInterface.cpp
//  *
//  *  Created on: July 16, 2020
//  *      Author: Quincy Jones
//  *
//  * Copyright (c) <2020> <Quincy Jones - quincy@implementedrobotics.com/>
//  * Permission is hereby granted, free of charge, to any person obtaining a
//  * copy of this software and associated documentation files (the "Software"),
//  * to deal in the Software without restriction, including without limitation
//  * the rights to use, copy, modify, merge, publish, distribute, sublicense,
//  * and/or sell copies of the Software, and to permit persons to whom the Software
//  * is furnished to do so, subject to the following conditions:
//  * The above copyright notice and this permission notice shall be included in all
//  * copies or substantial portions of the Software.
//  * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
//  * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
//  * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//  */

// // Primary Include
// #include <Nomad/Interface/SimulationInterface.hpp>

// // C System Includes

// // C++ System Includes
// #include <iostream>
// #include <string>
// #include <sstream>
// #include <memory>

// // Third-Party Includes
// #include <zcm/zcm-cpp.hpp>

// // Project Includes
// #include <Realtime/RealTimeTask.hpp>
// #include <Common/Time.hpp>

// namespace Robot
// {
//     namespace Nomad
//     {
//         namespace Interface
//         {

//             SimulationInterface::SimulationInterface(const std::string &name,
//                                                      const long rt_period,
//                                                      unsigned int rt_priority,
//                                                      const int rt_core_id,
//                                                      const unsigned int stack_size) : Realtime::RealTimeTaskNode(name, rt_period, rt_priority, rt_core_id, stack_size)
//             {
//                 // Create Ports
//                 // Sim Inputs
//                 input_port_map_[InputPort::SIM_DATA] = Communications::Port::CreateInput<sim_data_t>("SIM_DATA", rt_period_);

//                 // Control Inputs
//                 input_port_map_[InputPort::JOINT_CONTROL_CMD_IN] = Communications::Port::CreateInput<joint_control_cmd_t>("JOINT_CONTROL", rt_period_);

//                 // Outputs
//                 output_port_map_[OutputPort::IMU_STATE] = Communications::Port::CreateOutput("IMU_STATE", rt_period_);
//                 output_port_map_[OutputPort::JOINT_STATE] = Communications::Port::CreateOutput("JOINT_STATE", rt_period_);
//                 output_port_map_[OutputPort::COM_STATE] = Communications::Port::CreateOutput("POSE_STATE", rt_period_);
//                 output_port_map_[OutputPort::JOINT_CONTROL_CMD_OUT] = Communications::Port::CreateOutput("JOINT_CONTROL", rt_period_);
//             }

//             void SimulationInterface::Run()
//             {
//                 static uint64_t last_control_time = 0;
//                 static int lost_control = 0;

//                 // Systems::Time t;
//                 // if(!GetInputPort(InputPort::JOINT_CONTROL_CMD_IN)->Receive(joint_command_, std::chrono::microseconds(10)))
//                 // {
//                 //    //std::cout << "FAILED TO GET SYNCED JOINT CONTROL MESSAGE!!: " << lost_control++ << std::endl;
//                 //     // TODO: What to do.
//                 // }

//                 //std::cout << com_state_out_.pos[2] << std::endl;

//                 // Publish/Forward Outputs from Sim
//                 //GetOutputPort(OutputPort::IMU_STATE_OUT)->Send(imu_data_out_);
//                 //GetOutputPort(OutputPort::JOINT_STATE_OUT)->Send(joint_state_out_);
//                 //GetOutputPort(OutputPort::COM_STATE_OUT)->Send(com_state_out_);

//                 //last_control_time = Systems::Time::GetTimeStamp();

//                 // Publish/Forward to Sim
//                 // GetOutputPort(OutputPort::JOINT_CONTROL_CMD_OUT)->Send(joint_command_);
//                 //std::cout << "Command Out Time: " << joint_command_.timestamp << std::endl;
//                 //std::cout << "Turn around time: " << joint_command_.timestamp - last_control_time << std::endl;

//                 // Publish/Forward to Sim
//                 //memset(&joint_command_, 0, sizeof(joint_control_cmd_t));
//                 {
//                     Systems::Time t;
//                     GetOutputPort(OutputPort::JOINT_CONTROL_CMD_OUT)->Send(joint_command_);
//                     //std::cout << "OUT SEND: " << std::endl;
//                 }

//                 //imu_data_t imu_data_out_ = imu_data_;
//                 //com_state_t com_state_out_ = com_state_;
//                 //joint_state_t joint_state_out_ = joint_state_;

//                 // Read Plant State
//                 //if(!GetInputPort(InputPort::SIM_DATA)->Receive(sim_data_, std::chrono::microseconds(100)))
//                 {
//                     //    std::cout << "FAILED TO GET SYNCED SIM MESSAGE!!: " << lost_control++ << std::endl;
//                     // TODO: What to do.
//                 }
//                 //com_state_.pos = sim_data_.com_pos;
//                 // memcpy(com_state_.pos, sim_data_.com_pos, sizeof(double)*3);
//                 // memcpy(com_state_.theta, sim_data_.com_theta, sizeof(double)*3);
//                 // memcpy(com_state_.vel, sim_data_.com_vel, sizeof(double)*3);
//                 // memcpy(com_state_.omega, sim_data_.com_omega, sizeof(double)*3);
//                 // memcpy(com_state_.orientation, sim_data_.com_orientation, sizeof(double)*4);

//                 // memcpy(joint_state_.q, sim_data_.q, sizeof(double)*12);
//                 // memcpy(joint_state_.q_dot, sim_data_.q_dot, sizeof(double)*12);
//                 // memcpy(joint_state_.tau, sim_data_.tau, sizeof(double)*12);

//                 // GetOutputPort(OutputPort::COM_STATE)->Send(com_state_);
//                 //  GetOutputPort(OutputPort::JOINT_STATE)->Send(joint_state_);

//                 std::cout << "Loop Frequency: " << 1000000.0 / (double)(Systems::Time::GetTimeStamp() - last_control_time) << std::endl;
//                 last_control_time = Systems::Time::GetTimeStamp();
//             }

//             void SimulationInterface::Setup()
//             {
//                 bool outputs_bound = true;
//                 for (int i = 0; i < OutputPort::NUM_OUTPUTS; i++) // Bind all of our output ports
//                 {
//                     if (!GetOutputPort(i)->Bind())
//                     {
//                         outputs_bound = false;
//                     }
//                 }

//                 std::cout << "[SimulationInterface]: "
//                           << "Simulation Interface Publisher Running!: " << outputs_bound << " " << std::endl;
//             }

//         } // namespace Interface
//     }     // namespace Nomad
// } // namespace Robot
