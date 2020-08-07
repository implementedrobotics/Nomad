/*
 * LegController.cpp
 *
 *  Created on: June 28, 2020
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
#include <Controllers/LegController.hpp>

// C System Includes

// C++ System Includes
#include <iostream>
#include <string>
#include <sstream>
#include <memory>

// Third-Party Includes
#include <zcm/zcm-cpp.hpp>
#include <dart/utils/urdf/DartLoader.hpp>

// Project Includes
#include <Realtime/RealTimeTask.hpp>
#include <Communications/Messages/double_vec_t.hpp>
#include <Communications/Messages/generic_msg_t.hpp>
#include <Common/Time.hpp>

namespace Controllers::Locomotion
{
    LegController::LegController(const std::string &name,
                                 const long rt_period,
                                 unsigned int rt_priority,
                                 const int rt_core_id,
                                 const unsigned int stack_size) : Realtime::RealTimeTaskNode(name, rt_period, rt_priority, rt_core_id, stack_size),
                                                                  num_legs_(4), num_dofs_(3)
    {
        // Member Variables //
        total_dofs_ = num_legs_ * num_dofs_;

        // PD Gain Variables
        k_P_cartesian_ = Eigen::MatrixXd(total_dofs_, total_dofs_);
        k_D_cartesian_ = Eigen::MatrixXd(total_dofs_, total_dofs_);
        k_P_joint_ = Eigen::MatrixXd(total_dofs_, total_dofs_);
        k_D_joint_ = Eigen::MatrixXd(total_dofs_, total_dofs_);

        // Foot Position Variable
        foot_pos_ = Eigen::VectorXd::Zero(total_dofs_);
        foot_vel_ = Eigen::VectorXd::Zero(total_dofs_);

        // Jacobian Variable (Linear or Angular 3, Full 6)
        J_ = Eigen::MatrixXd(num_legs_ * 3, total_dofs_);

        // Create Input/Output Messages
        // leg_command_input_.length = sizeof(leg_controller_cmd_t);
        // leg_command_input_.data.resize(leg_command_input_.length);

        // servo_command_output_.length = sizeof(leg_controller_cmd_t);
        // servo_command_output_.data.resize(servo_command_output_.length);

        // Create Ports
        // Leg Controller Input Port
        input_port_map_[InputPort::LEG_COMMAND] = Communications::Port::CreateInput<leg_controller_cmd_t>("LEG_COMMAND", rt_period_);

        // Leg Controller Output Ports
        output_port_map_[OutputPort::SERVO_COMMAND] = Communications::Port::CreateOutput("SERVO_COMMAND", rt_period_);
    }

    void LegController::Run()
    {
        // Zero Force Outputs
        Eigen::VectorXd tau_output = Eigen::VectorXd::Zero(total_dofs_);
        Eigen::VectorXd force_output = Eigen::VectorXd::Zero(total_dofs_);
        Eigen::VectorXd foot_pos_desired = Eigen::VectorXd::Zero(total_dofs_);
        Eigen::VectorXd foot_vel_desired = Eigen::VectorXd::Zero(total_dofs_);

        q_out_ = Eigen::VectorXd::Zero(total_dofs_);
        q_d_out_ = Eigen::VectorXd::Zero(total_dofs_);

        // Reset State, Zero Inputs, and Force/Torque outputs
        // ResetState();

        // Read Command
        if (GetInputPort(InputPort::LEG_COMMAND)->Receive(leg_command_input_))
        {
            //std::cout << "Got: " << imu_data_.accel[2] << std::endl;
        }

        // Setup Vars
        // Read any Feed Forwards
        tau_output = Eigen::Map<Eigen::VectorXd>(leg_command_input_.torque_ff, total_dofs_);
        force_output = Eigen::Map<Eigen::VectorXd>(leg_command_input_.force_ff, total_dofs_);

        // Gains
        k_P_cartesian_ = Eigen::Map<Eigen::VectorXd>(leg_command_input_.k_p_cartesian, total_dofs_).asDiagonal();
        k_D_cartesian_ = Eigen::Map<Eigen::VectorXd>(leg_command_input_.k_d_cartesian, total_dofs_).asDiagonal();
        k_P_joint_ = Eigen::Map<Eigen::VectorXd>(leg_command_input_.k_p_joint, total_dofs_).asDiagonal();
        k_D_joint_ = Eigen::Map<Eigen::VectorXd>(leg_command_input_.k_d_joint, total_dofs_).asDiagonal();

        //std::cout << "FF: " << force_output << std::endl;
        //std::cout << "KP: " << Eigen::Map<Eigen::VectorXd>(leg_command_input_.k_p_cartesian, total_dofs_) << std::endl;

        // Current Foot position and velocity
        foot_pos_ = Eigen::Map<Eigen::VectorXd>(leg_command_input_.foot_pos, total_dofs_);
        foot_vel_ = Eigen::Map<Eigen::VectorXd>(leg_command_input_.foot_vel, total_dofs_);

        // Desired Foot position and velocity
        foot_pos_desired = Eigen::Map<Eigen::VectorXd>(leg_command_input_.foot_pos_desired, total_dofs_);
        foot_vel_desired = Eigen::Map<Eigen::VectorXd>(leg_command_input_.foot_vel_desired, total_dofs_);

        // Compute Forces
        force_output += k_P_cartesian_ * (foot_pos_desired - foot_pos_);
        force_output += k_D_cartesian_ * (foot_vel_desired - foot_vel_);

        Eigen::VectorXd force_to_tau = J_.transpose() * force_output;

        // Add Feed Forward Forces to Torque Feed Forwards
        // TODO: Need to compute Jacobian
        tau_output += (J_.transpose() * force_output);

        // Sync Vectors

        // Publish Forces

        // Publish State
        //bool send_status = GetOutputPort(OutputPort::STATE_HAT)->Send(output_state_);

        // std::cout << "[LegController]: Publishing: " << std::endl; //output_state_.data[Idx::X] << " Send: " << send_status << std::endl;

    }

    void LegController::Setup()
    {
        // TODO: Validate Ports being valid etc.  Error check

        bool outputs_bound = true;
        for (int i = 0; i < NUM_OUTPUTS; i++) // Bind all of our output ports
        {
            if (!GetOutputPort(i)->Bind())
            {
                outputs_bound = false;
            }
        }
        std::cout << "[LegController]: "
                  << "Leg Controller Publisher Running!: "
                  << "[OUTPUTS]: " << outputs_bound << std::endl;
    }

    void LegController::ResetState()
    {
        //  memset(&leg_command_input_, 0, sizeof(leg_controller_cmd_t));
        //  memset(&servo_command_output_, 0, sizeof(leg_controller_cmd_t));
    }

} // namespace Controllers::Locomotion
