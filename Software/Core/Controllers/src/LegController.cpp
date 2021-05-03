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

// Project Includes
#include <Communications/Messages/double_vec_t.hpp>
#include <Communications/Messages/generic_msg_t.hpp>
#include <Common/Time.hpp>

namespace Controllers::Locomotion
{
    LegController::LegController(const double T_s)  : SystemBlock("Leg_Kin_State_Estimator_Task", T_s),
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

        // Create Ports
        // Leg Controller Input Port
        input_port_map_[InputPort::LEG_COMMAND] = Communications::Port<leg_controller_cmd_t>::CreateInput("LEG_COMMAND");
        
        // Leg Controller Output Ports
        output_port_map_[OutputPort::SERVO_COMMAND] = Communications::Port<joint_control_cmd_t>::CreateOutput("SERVO_COMMAND");
    }

    // Update function for stateful outputs
    void LegController::UpdateStateOutputs()
    {
        // Receive Data
    }

    // Update function for stateless outputs
    void LegController::UpdateStatelessOutputs()
    {
        // Zero Force Outputs
        Eigen::VectorXd tau_output = Eigen::VectorXd::Zero(total_dofs_);
        Eigen::VectorXd force_output = Eigen::VectorXd::Zero(total_dofs_);
        Eigen::VectorXd foot_pos_desired = Eigen::VectorXd::Zero(total_dofs_);
        Eigen::VectorXd foot_vel_desired = Eigen::VectorXd::Zero(total_dofs_);

        q_out_ = Eigen::VectorXd::Zero(total_dofs_);
        q_d_out_ = Eigen::VectorXd::Zero(total_dofs_);

        // Reset State, Zero Inputs, and Force/Torque outputs
        //ResetState();

        // TODO: Wait for some timeout, if control deadline missed -> ZERO OUTPUTS
        // Read Command
        bool input = GetInputPort(InputPort::LEG_COMMAND)->Receive(leg_command_input_);

        //std::cout << "INPUT: " << input << " | " << leg_command_input_.sequence_num << std::endl;

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

        // Contact Jacobians
        J_ = Eigen::Map<Eigen::MatrixXd>(leg_command_input_.J_c, J_.rows(), J_.cols());

        
        //std::cout << J_ << std::endl;

        // Compute Forces
        force_output += k_P_cartesian_ * (foot_pos_desired - foot_pos_);
        force_output += k_D_cartesian_ * (foot_vel_desired - foot_vel_);

        //std::cout << "Force: " << force_output << std::endl;

        Eigen::VectorXd force_to_tau = J_.transpose() * force_output;
        
        tau_output += force_to_tau;

        Eigen::Map<Eigen::VectorXd>(servo_command_output_.tau_ff, 12) = tau_output;
        // static double max = 0.0;
        // static double min = 10000000;
        // max = std::max(max, tau_output.maxCoeff());
        // min = std::min(min, tau_output.minCoeff());

        // std::cout << " MAX TORQUE: " << max << std::endl;
        // std::cout << " MIN TORQUE: " << min << std::endl;
        //std::cout << "Error: " << (foot_pos_desired - foot_pos_) <<std::endl;
        //std::cout << "Force: " << force_output <<std::endl;
        //std::cout << "in leg Send: " << tau_output <<std::endl;
        // Add Feed Forward Forces to Torque Feed Forwards
        //tau_output += (J_.transpose() * force_output);

        //servo_command_output_.tau_ff[1] = 5;

        // Publish Forces
        GetOutputPort(OutputPort::SERVO_COMMAND)->Send(servo_command_output_);

        //std::cout << "[LegController]: Publishing: " << std::endl;
    }

    // Update function for next state from inputs
    void LegController::UpdateState()
    {

    }

    void LegController::ResetState()
    {
        memset(&leg_command_input_, 0, sizeof(leg_controller_cmd_t));
        memset(&servo_command_output_, 0, sizeof(servo_command_output_));
    }

} // namespace Controllers::Locomotion
