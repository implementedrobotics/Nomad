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

namespace Controllers
{
    namespace Locomotion
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
            leg_command_input_.length = sizeof(leg_controller_cmd_t);
            leg_command_input_.data.resize(leg_command_input_.length);

            servo_command_output_.length = sizeof(leg_controller_cmd_t);
            servo_command_output_.data.resize(servo_command_output_.length);

            // Create Ports
            // Leg Controller Input Port
            input_port_map_[InputPort::LEG_COMMAND] = std::make_shared<Realtime::Port>("LEG_COMMAND", Realtime::Port::Direction::INPUT, Realtime::Port::DataType::BYTE, 1, rt_period_);

            // Leg Controller Output Ports
            output_port_map_[OutputPort::SERVO_COMMAND] = std::make_shared<Realtime::Port>("SERVO_COMMAND", Realtime::Port::Direction::OUTPUT, Realtime::Port::DataType::BYTE, 1, rt_period);
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
            bool success = GetInputPort(InputPort::LEG_COMMAND)->Receive(leg_command_input_);

            Controllers::Locomotion::leg_controller_cmd_t leg_controller_cmd_;
            memcpy(&leg_controller_cmd_, leg_command_input_.data.data(), sizeof(Controllers::Locomotion::leg_controller_cmd_t));

            // Setup Vars
            // Read any Feed Forwards
            tau_output = Eigen::Map<Eigen::VectorXd>(leg_controller_cmd_.torque_ff, total_dofs_);
            force_output = Eigen::Map<Eigen::VectorXd>(leg_controller_cmd_.force_ff, total_dofs_);

            std::cout << "FF: " << force_output << std::endl;
            //     // Gains
            //     k_P_cartesian_ = Eigen::Map<Eigen::VectorXd>(input_desired_[InputPort::K_P_CARTESIAN].data.data(), total_dofs_).asDiagonal();
            //     k_D_cartesian_ = Eigen::Map<Eigen::VectorXd>(input_desired_[InputPort::K_D_CARTESIAN].data.data(), total_dofs_).asDiagonal();
            //     k_P_joint_ = Eigen::Map<Eigen::VectorXd>(input_desired_[InputPort::K_P_JOINT].data.data(), total_dofs_).asDiagonal();
            //     k_D_joint_ = Eigen::Map<Eigen::VectorXd>(input_desired_[InputPort::K_D_JOINT].data.data(), total_dofs_).asDiagonal();

            //     // Desired Foot position and velocity
            //     foot_pos_desired = Eigen::Map<Eigen::VectorXd>(input_desired_[InputPort::FOOT_POSITION].data.data(), total_dofs_);
            //     foot_vel_desired = Eigen::Map<Eigen::VectorXd>(input_desired_[InputPort::FOOT_VELOCITY].data.data(), total_dofs_);

            //     // Compute Forces
            //     force_output += k_P_cartesian_ * (foot_pos_desired - foot_pos_);
            //     force_output += k_D_cartesian_ * (foot_vel_desired - foot_vel_);

            //     Eigen::VectorXd force_to_tau = J_.transpose() * force_output;

            //     // Add Feed Forward Forces to Torque Feed Forwards
            //     // TODO: Need to compute Jacobian
            //     tau_output += (J_.transpose() * force_output);

            //     // Sync Vectors

            //     // Publish Forces

            //     // Read/Update State

            //     // Publish Updated State
            //     Eigen::VectorXd force_ff_in = Eigen::Map<Eigen::VectorXd>(input_desired_[InputPort::FORCE_FF].data.data(), total_dofs_);
            //     Eigen::VectorXd torque_ff_in = Eigen::Map<Eigen::VectorXd>(input_desired_[InputPort::TORQUE_FF].data.data(), total_dofs_);

            //     //memcpy(reference_out_.data.data(), X_ref_.data(), sizeof(double) * X_ref_.size());
            //     // Eigen::VectorXd x_hat_ = Eigen::Map<Eigen::VectorXd>(x_hat_in_.data.data(), num_states_);
            //     // //std::cout << "[StateEstimator]: Received: " << x_hat_in_.sequence_num <<  std::endl;

            //     // // Update State
            //     // output_state_.data[Idx::X] = x_hat_[0]; // X Position
            //     // output_state_.data[Idx::Y] = x_hat_[1]; // Y Position
            //     // output_state_.data[Idx::Z] = x_hat_[2]; // Z Position
            //     // output_state_.data[Idx::X_DOT] = x_hat_[3]; // X Velocity
            //     // output_state_.data[Idx::Y_DOT] = x_hat_[4]; // Y Velocity
            //     // output_state_.data[Idx::Z_DOT] = x_hat_[5]; // Z Velocity
            //     // output_state_.data[Idx::PHI] = x_hat_[6]; // Roll Orientation
            //     // output_state_.data[Idx::THETA] = x_hat_[7]; // Pitch Orientation
            //     // output_state_.data[Idx::PSI] = x_hat_[8]; // Yaw Orientation
            //     // output_state_.data[Idx::W_X] = x_hat_[9]; // Roll Rate
            //     // output_state_.data[Idx::W_Y] = x_hat_[10]; // Pitch Rate
            //     // output_state_.data[Idx::W_Z] = x_hat_[11]; // Yaw Rate
            //     // output_state_.data[Idx::GRAVITY] = x_hat_[12]; // Gravity

            //     // //std::cout << "State Estimator Send: " << std::endl;

            //     // Publish State
            //     //bool send_status = GetOutputPort(OutputPort::STATE_HAT)->Send(output_state_);

            //    // std::cout << "[LegController]: Publishing: " << std::endl; //output_state_.data[Idx::X] << " Send: " << send_status << std::endl;

            //    // Update State
            //    //J_ = robot_->getLinearJacobian(foot, body);
        }

        void LegController::Setup()
        {
            // TODO: Validate Ports being valid etc.  Error check

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
            std::cout << "[LegController]: "
                      << "Leg Controller Publisher Running!: "
                      << "[INPUTS]: " << inputs_connected << "[OUTPUTS]: " << outputs_bound << std::endl;
        }

        void LegController::ResetState()
        {
            //  memset(&leg_command_input_, 0, sizeof(leg_controller_cmd_t));
            //  memset(&servo_command_output_, 0, sizeof(leg_controller_cmd_t));
        }

        void LegController::SetRobotSkeleton(dart::dynamics::SkeletonPtr robot)
        {
            robot_ = robot;
        }
    } // namespace Locomotion
} // namespace Controllers
