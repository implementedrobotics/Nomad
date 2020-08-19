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

// Project Includes
const std::string sim_url = "udpm://239.255.76.67:7667?ttl=0";

namespace Robot::Nomad::Interface
{

    SimulationInterface::SimulationInterface(const double T_s)  : SystemBlock("Nomad_Simulation_Interface", T_s)
    {
        // Create Ports
        // Sim Inputs
        std::shared_ptr<Communications::Port> sim_in = Communications::Port::CreateInput<sim_data_t>("SIM_DATA");
        sim_in->SetTransport(Communications::Port::TransportType::UDP, sim_url, "nomad.sim.data");
        // TODO: SetPortInput
        input_port_map_[InputPort::SIM_DATA] = sim_in;

        // Control Inputs
        input_port_map_[InputPort::JOINT_CONTROL_CMD_IN] = Communications::Port::CreateInput<joint_control_cmd_t>("JOINT_CONTROL");

        // Outputs
        output_port_map_[OutputPort::IMU_DATA] = Communications::Port::CreateOutput("IMU_DATA");
        output_port_map_[OutputPort::JOINT_STATE] = Communications::Port::CreateOutput("JOINT_STATE");
        output_port_map_[OutputPort::COM_STATE] = Communications::Port::CreateOutput("POSE_STATE");
        output_port_map_[OutputPort::JOINT_CONTROL_CMD_OUT] = Communications::Port::CreateOutput("JOINT_CONTROL");

        SetPortOutput(SimulationInterface::JOINT_CONTROL_CMD_OUT,  Communications::Port::TransportType::UDP, sim_url, "nomad.sim.joint_cmd");
    }

    // Update function for stateful outputs
    void SimulationInterface::UpdateStateOutputs()
    {
        // Receive Data
    }

    // Update function for stateless outputs
    void SimulationInterface::UpdateStatelessOutputs()
    {
        static uint64_t last_control_time = 0; 
        static int lost_control = 0;

        // Get Input Leg Command
        GetInputPort(InputPort::JOINT_CONTROL_CMD_IN)->Receive(joint_command_);
        //std::cout << "Got Input!! - " << joint_command_.sequence_num << std::endl;
        // Send Data

        // Publish/Forward to Sim
        //memset(&joint_command_, 0, sizeof(joint_control_cmd_t));
        //{
            //Systems::Time t;
            GetOutputPort(OutputPort::JOINT_CONTROL_CMD_OUT)->Send(joint_command_);
            //std::cout << "OUT SEND: " << std::endl;
        //}

        // Receive Data
        //std::cout << "RUNNING" << std::endl;
        GetInputPort(InputPort::SIM_DATA)->Receive(sim_data_, std::chrono::microseconds(5000));

        memcpy(com_state_.pos, sim_data_.com_pos, sizeof(double) * 3);
        memcpy(com_state_.theta, sim_data_.com_theta, sizeof(double)*3);
        memcpy(com_state_.vel, sim_data_.com_vel, sizeof(double)*3);
        memcpy(com_state_.omega, sim_data_.com_omega, sizeof(double)*3);
        memcpy(com_state_.orientation, sim_data_.com_orientation, sizeof(double)*4);

        memcpy(imu_data_.omega, sim_data_.imu_omega, sizeof(double)*3);
        memcpy(imu_data_.orientation, sim_data_.imu_orientation, sizeof(double)*4);
        memcpy(imu_data_.accel, sim_data_.imu_accel, sizeof(double)*3);

        memcpy(joint_state_.q, sim_data_.q, sizeof(double)*12);
        memcpy(joint_state_.q_dot, sim_data_.q_dot, sizeof(double)*12);
        memcpy(joint_state_.tau, sim_data_.tau, sizeof(double)*12);

        GetOutputPort(OutputPort::COM_STATE)->Send(com_state_);
        GetOutputPort(OutputPort::IMU_DATA)->Send(imu_data_);
        GetOutputPort(OutputPort::JOINT_STATE)->Send(joint_state_);


        //std::cout << sim_data_.sequence_num;
        //std::cout << "Loop Frequency: " << 1000000.0 / (double)(Systems::Time::GetTimeStamp() - last_control_time) << " AND " << got << std::endl;
        last_control_time = Systems::Time::GetTimeStamp();
        
    }

    // Update function for next state from inputs
    void SimulationInterface::UpdateState()
    {

    }

    // void SimulationInterface::Run()
    // {
    //     static uint64_t last_control_time = 0;
    //     static int lost_control = 0;

    //     // Systems::Time t;
    //     // if(!GetInputPort(InputPort::JOINT_CONTROL_CMD_IN)->Receive(joint_command_, std::chrono::microseconds(10)))
    //     // {
    //     //    //std::cout << "FAILED TO GET SYNCED JOINT CONTROL MESSAGE!!: " << lost_control++ << std::endl;
    //     //     // TODO: What to do.
    //     // }

    //     //std::cout << com_state_out_.pos[2] << std::endl;

    //     // Publish/Forward Outputs from Sim
    //     //GetOutputPort(OutputPort::IMU_STATE_OUT)->Send(imu_data_out_);
    //     //GetOutputPort(OutputPort::JOINT_STATE_OUT)->Send(joint_state_out_);
    //     //GetOutputPort(OutputPort::COM_STATE_OUT)->Send(com_state_out_);

    //     //last_control_time = Systems::Time::GetTimeStamp();

    //     // Publish/Forward to Sim
    //     // GetOutputPort(OutputPort::JOINT_CONTROL_CMD_OUT)->Send(joint_command_);
    //     //std::cout << "Command Out Time: " << joint_command_.timestamp << std::endl;
    //     //std::cout << "Turn around time: " << joint_command_.timestamp - last_control_time << std::endl;

    //     // Publish/Forward to Sim
    //     //memset(&joint_command_, 0, sizeof(joint_control_cmd_t));
    //     {
    //         Systems::Time t;
    //         GetOutputPort(OutputPort::JOINT_CONTROL_CMD_OUT)->Send(joint_command_);
    //         //std::cout << "OUT SEND: " << std::endl;
    //     }

    //     //imu_data_t imu_data_out_ = imu_data_;
    //     //com_state_t com_state_out_ = com_state_;
    //     //joint_state_t joint_state_out_ = joint_state_;

    //     // Read Plant State
    //     //if(!GetInputPort(InputPort::SIM_DATA)->Receive(sim_data_, std::chrono::microseconds(100)))
    //     {
    //         //    std::cout << "FAILED TO GET SYNCED SIM MESSAGE!!: " << lost_control++ << std::endl;
    //         // TODO: What to do.
    //     }
    //     //com_state_.pos = sim_data_.com_pos;
    //     // memcpy(com_state_.pos, sim_data_.com_pos, sizeof(double)*3);
    //     // memcpy(com_state_.theta, sim_data_.com_theta, sizeof(double)*3);
    //     // memcpy(com_state_.vel, sim_data_.com_vel, sizeof(double)*3);
    //     // memcpy(com_state_.omega, sim_data_.com_omega, sizeof(double)*3);
    //     // memcpy(com_state_.orientation, sim_data_.com_orientation, sizeof(double)*4);

    //     // memcpy(joint_state_.q, sim_data_.q, sizeof(double)*12);
    //     // memcpy(joint_state_.q_dot, sim_data_.q_dot, sizeof(double)*12);
    //     // memcpy(joint_state_.tau, sim_data_.tau, sizeof(double)*12);

    //     // GetOutputPort(OutputPort::COM_STATE)->Send(com_state_);
    //     //  GetOutputPort(OutputPort::JOINT_STATE)->Send(joint_state_);

    //     std::cout << "Loop Frequency: " << 1000000.0 / (double)(Systems::Time::GetTimeStamp() - last_control_time) << std::endl;
    //     last_control_time = Systems::Time::GetTimeStamp();
    // }

} // namespace Robot::Nomad::Interface
