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

namespace Robot::Nomad::Dynamics
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

        // Create Ports
        // Input Ports
        input_port_map_[InputPort::BODY_STATE_HAT] = Communications::Port::CreateInput<com_state_t>("BODY_STATE_HAT", rt_period_);
        input_port_map_[InputPort::JOINT_STATE] = Communications::Port::CreateInput<joint_state_t>("JOINT_STATE", rt_period_);

        // Output Ports
        output_port_map_[OutputPort::FULL_STATE] = Communications::Port::CreateOutput("FULL_STATE", rt_period_);
    }

    void NomadDynamics::Run()
    {
        // TODO: Time this function  Would be nice if we could run this at 2x sample rate
        // Read Inputs
        if (!GetInputPort(InputPort::BODY_STATE_HAT)->Receive(com_state_, std::chrono::microseconds(1000000)))
        {
            std::cout << "[DYNAMICS]: FAILED TO GET SYNCED BODY STATE MESSAGE!!: " << std::endl;
            // TODO: What to do.
        }

        if (!GetInputPort(InputPort::JOINT_STATE)->Receive(joint_state_, std::chrono::microseconds(1000000)))
        {
            std::cout << "[DYNAMICS]: FAILED TO GET SYNCED JOINT STATE MESSAGE!!: " << std::endl;
            // TODO: What to do.
        }

        //std::cout << "JOINT STATE: " << joint_state_.sequence_num << std::endl;
        //std::cout << "BODY STATE: " << com_state_.sequence_num << std::endl;

        // Setup some state vectors
        Eigen::VectorXd q = Eigen::VectorXd::Zero(kNumTotalDofs);
        Eigen::VectorXd q_dot = Eigen::VectorXd::Zero(kNumTotalDofs);
        Eigen::VectorXd tau = Eigen::VectorXd::Zero(kNumTotalDofs);

        // Copy into our state vectors
        q.segment(0, 3) = Eigen::Map<Eigen::VectorXd>(com_state_.theta, 3);
        q.segment(3, 3) = Eigen::Map<Eigen::VectorXd>(com_state_.pos, 3);
        q.tail(kNumActuatedDofs) = Eigen::Map<Eigen::VectorXd>(joint_state_.q, kNumActuatedDofs);

        q_dot.segment(0, 3) = Eigen::Map<Eigen::VectorXd>(com_state_.omega, 3);
        q_dot.segment(3, 3) = Eigen::Map<Eigen::VectorXd>(com_state_.vel, 3);
        q_dot.tail(kNumActuatedDofs) = Eigen::Map<Eigen::VectorXd>(joint_state_.q_dot, kNumActuatedDofs);

        tau.tail(kNumActuatedDofs) = Eigen::Map<Eigen::VectorXd>(joint_state_.tau, kNumActuatedDofs);

        //std::cout << "Got Pos 1: " << q << std::endl;
        //std::cout << "Got Vel: " << q_dot << std::endl;
        robot_->setPositions(q);
        robot_->setVelocities(q_dot);
        robot_->setForces(tau);

        // Update Dynamics State
        robot_->computeForwardKinematics();
        robot_->computeForwardDynamics();
        
        // Setup our Augemented Contact Jacobian
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

        //std::cout << "VELS: " << robot_->getVelocities().head(6) << std::endl;
        //std::cout << "Size: " << (J_legs_ * robot_->getVelocities()) << std::endl;
        //std::cout << "Row: " << (J_legs_ * robot_->getVelocities()).rows() << std::endl;
        //std::cout << "Col: " << (J_legs_ * robot_->getVelocities()).cols() << std::endl;
        // std::cout << std::setprecision (3) << std::fixed << "Jacobian: " << " [" << J_legs_.rows() << " , " << J_legs_.cols() << "]:" << std::endl << J_legs_ << std::endl;
        // std::cout << " End " << std::endl;

        // Publish Leg Command
        bool send_status = GetOutputPort(OutputPort::FULL_STATE)->Send(full_state_);

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
} // namespace Robot::Nomad::Dynamics
