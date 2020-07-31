/*
 * NomadDynamics.cpp
 *
 *  Created on: July 2, 2020
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
#include <Nomad/NomadDynamics.hpp>

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
#include <Controllers/LegController.hpp>
#include <Communications/Messages/double_vec_t.hpp>
#include <Communications/Messages/generic_msg_t.hpp>
#include <Common/Time.hpp>

namespace Robot
{
    namespace Nomad
    {
        namespace Dynamics
        {

            NomadDynamics::NomadDynamics(const std::string &name,
                                         const long rt_period,
                                         unsigned int rt_priority,
                                         const int rt_core_id,
                                         const unsigned int stack_size) : Realtime::RealTimeTaskNode(name, rt_period, rt_priority, rt_core_id, stack_size)
            {

                // Matrix pre setup
                J_legs_ = Eigen::MatrixXd(3 * kNumContacts, kNumTotalDofs);

                // Selection MAtrices
                S_f_ = Eigen::MatrixXd::Zero(kNumFloatingDofs, kNumTotalDofs);
                S_f_.block(0, 0, kNumFloatingDofs, kNumTotalDofs).setIdentity();

                S_j_ = Eigen::MatrixXd::Zero(kNumActuatedDofs, kNumTotalDofs);
                S_j_.block(0, kNumFloatingDofs, kNumActuatedDofs, kNumActuatedDofs).setIdentity();

                // Create Output Messages
                // Full State Output Message
                nomad_full_state_msg_.length = sizeof(nomad_full_state_t);
                nomad_full_state_msg_.data.resize(nomad_full_state_msg_.length);

                // Intialize to 'zero' state
                memset(&full_state_, 0, sizeof(nomad_full_state_t));

                // Create Ports
                // Primary Controller Input Port
                input_port_map_[InputPort::BODY_STATE_HAT] = Realtime::Port::CreateInput<com_state_t>("BODY_STATE_HAT", rt_period_);
                input_port_map_[InputPort::JOINT_STATE] = Realtime::Port::CreateInput<joint_state_t>("JOINT_STATE", rt_period_);

                // Primary Controller Output Ports
                output_port_map_[OutputPort::FULL_STATE] = std::make_shared<Realtime::Port>("FULL_STATE", Realtime::Port::Direction::OUTPUT, Realtime::Port::DataType::BYTE, 1, rt_period);
            }

            void NomadDynamics::Run()
            {
                // Get Control Inputs, Modes, Trajectory etc
                 bool imu_recv = GetInputPort(InputPort::BODY_STATE_HAT)->Receive(com_state_); // Receive Setpoint

                if(GetInputPort(InputPort::JOINT_STATE)->Receive(joint_state_))
                {
                    
                }
                //  if (!imu_recv)
                //  {
                //      std::cout << "[NomadControl]: Receive Buffer Empty!" << std::endl;
                //      return;
                //  }

                 //std::cout << "RECEIVED: " << com_state_.pos[2] << std::endl;

                // Read Inputs
                // 1) Body State (State Estimator)
                // 2) Leg State (Plant Bridge Output)

                //bool receive = GetInputPort(InputPort::BODY_STATE)->Receive(control_mode_msg_);
                //bool receive = GetInputPort(InputPort::LEG_STATE)->Receive(control_mode_msg_);

                //robot_->setPositions()
                // Update Dynamics State
                robot_->computeForwardKinematics();
                robot_->computeForwardDynamics();

                // std::cout << " Start " << std::endl;

                // Setup our Jacobian
                // TODO: How to put this in the loop? Block Operation?
                // TODO: All foot positions/velocities and jacobians are in the hip frame.  Should we make these world frame?
                J_legs_ << robot_->getLinearJacobian(foot_body_[0], hip_base_body_[0]),
                    robot_->getLinearJacobian(foot_body_[1], hip_base_body_[1]),
                    robot_->getLinearJacobian(foot_body_[2], hip_base_body_[2]),
                    robot_->getLinearJacobian(foot_body_[3], hip_base_body_[3]);

                // Copy Data over for our Full Robot State Message

                Eigen::Map<Eigen::MatrixXd>(full_state_.J_c, 3 * kNumContacts, kNumTotalDofs) = J_legs_;
                Eigen::Map<Eigen::VectorXd>(full_state_.q, kNumTotalDofs) = robot_->getPositions();
                Eigen::Map<Eigen::VectorXd>(full_state_.q_dot, kNumTotalDofs) = robot_->getVelocities();
                Eigen::Map<Eigen::MatrixXd>(full_state_.M, kNumTotalDofs, kNumTotalDofs) = robot_->getMassMatrix();
                Eigen::Map<Eigen::VectorXd>(full_state_.b, kNumTotalDofs) = robot_->getCoriolisForces();
                Eigen::Map<Eigen::VectorXd>(full_state_.g, kNumTotalDofs) = robot_->getGravityForces();
                Eigen::Map<Eigen::VectorXd>(full_state_.foot_vel, kNumActuatedDofs) = (J_legs_.rightCols(12) * robot_->getVelocities().tail(12));

                // Compute Foot Positions
                for (int i = 0; i < NUM_LEGS; i++)
                {
                    // Foot Position
                    Eigen::Map<Eigen::Vector3d>(&full_state_.foot_pos[i * 3], 3) = foot_body_[i]->getTransform(hip_base_body_[i]).translation();
                }

                //std::cout << "VELS: " << robot_->getVelocities() << std::endl;
                //std::cout << "Size: " << (J_legs_ * robot_->getVelocities()) << std::endl;
                //std::cout << "Row: " << (J_legs_ * robot_->getVelocities()).rows() << std::endl;
                //std::cout << "Col: " << (J_legs_ * robot_->getVelocities()).cols() << std::endl;
                // std::cout << std::setprecision (3) << std::fixed << "Jacobian: " << " [" << J_legs_.rows() << " , " << J_legs_.cols() << "]:" << std::endl << J_legs_ << std::endl;
                // std::cout << " End " << std::endl;

                //memcpy(full_state_.b, robot_->getCoriolisForces().data(), sizeof(double) * robot_->getCoriolisForces().size());

                //full_state_.b = robot_->getCoriolisForces();
                //full_state_.g = robot_->getGravityForces();
                // Copy Full State to Output Message
                memcpy(nomad_full_state_msg_.data.data(), &full_state_, sizeof(nomad_full_state_t));
                // Publish Leg Command
                // bool send_status = GetOutputPort(OutputPort::FULL_STATE)->Send(nomad_full_state_msg_);

                //std::cout << "[NomadDynamics]: Publishing: Send: " << send_status << std::endl;
            }

            void NomadDynamics::Setup()
            {
                bool outputs_bound = true;
                for (int i = 0; i < NUM_OUTPUTS; i++) // Bind all of our output ports
                {
                    if (!GetOutputPort(i)->Bind())
                    {
                        outputs_bound = false;
                    }
                }

                std::cout << "[NomadDynamics]: "
                          << "Nomad Dynamics  Publisher Running!: " << outputs_bound << std::endl;
            }

            void NomadDynamics::SetRobotSkeleton(dart::dynamics::SkeletonPtr robot)
            {
                robot_ = robot;

                // Update Body Pointer for Floating Base Body
                base_body_ = robot_->getBodyNode("base_link");

                // Update Body Pointers for Hip Base(s)
                hip_base_body_[LegIdx::FRONT_LEFT] = robot_->getBodyNode("b_haa_motor_FL");
                hip_base_body_[LegIdx::FRONT_RIGHT] = robot_->getBodyNode("b_haa_motor_FR");
                hip_base_body_[LegIdx::REAR_LEFT] = robot_->getBodyNode("b_haa_motor_RL");
                hip_base_body_[LegIdx::REAR_RIGHT] = robot_->getBodyNode("b_haa_motor_RR");

                // Update Body Pointers for Foot Bodie(s)
                foot_body_[LegIdx::FRONT_LEFT] = robot_->getBodyNode("b_foot_FL");
                foot_body_[LegIdx::FRONT_RIGHT] = robot_->getBodyNode("b_foot_FR");
                foot_body_[LegIdx::REAR_LEFT] = robot_->getBodyNode("b_foot_RL");
                foot_body_[LegIdx::REAR_RIGHT] = robot_->getBodyNode("b_foot_RR");

                // Update DOF Pointers in skeleton
                for (int i = 0; i < DOFIdx::NUM_TOTAL_DOFS; i++)
                {
                    DOF_[i] = robot_->getDof(i);
                }
            }

        } // namespace Dynamics
    }     // namespace Nomad
} // namespace Robot
