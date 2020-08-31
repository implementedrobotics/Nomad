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
#include <dart/dynamics/FreeJoint.hpp>

// Project Includes
#include <Controllers/LegController.hpp>
#include <Communications/Messages/double_vec_t.hpp>
#include <Communications/Messages/generic_msg_t.hpp>
#include <Common/Time.hpp>
#include <Common/Math/MathUtils.hpp>

namespace Robot::Nomad::Dynamics
{
    NomadDynamics::NomadDynamics(const double T_s)  : SystemBlock("Nomad_Dynamics_Handler", T_s)
    {

        // Matrix pre setup
        J_legs_ = Eigen::MatrixXd(3 * kNumContacts, kNumTotalDofs);

        // Selection Matrices
        S_f_ = Eigen::MatrixXd::Zero(kNumFloatingDofs, kNumTotalDofs);
        S_f_.block(0, 0, kNumFloatingDofs, kNumTotalDofs).setIdentity();

        S_j_ = Eigen::MatrixXd::Zero(kNumActuatedDofs, kNumTotalDofs);
        S_j_.block(0, kNumFloatingDofs, kNumActuatedDofs, kNumActuatedDofs).setIdentity();

        // Create Ports
        // Input Ports
        input_port_map_[InputPort::BODY_STATE_HAT] = Communications::Port::CreateInput<com_state_t>("BODY_STATE_HAT");
        input_port_map_[InputPort::JOINT_STATE] = Communications::Port::CreateInput<joint_state_t>("JOINT_STATE");

        // Output Ports
        output_port_map_[OutputPort::FULL_STATE] = Communications::Port::CreateOutput("FULL_STATE");
    }

    void NomadDynamics::UpdateStateOutputs()
    {
        // Receive Data
    }

    // Update function for stateless outputs
    void NomadDynamics::UpdateStatelessOutputs()
    {
        // Receive Body COM State Estimate
        GetInputPort(InputPort::BODY_STATE_HAT)->Receive(com_state_);

        // Receive Joint State Estimate from Plant Interface
        GetInputPort(InputPort::JOINT_STATE)->Receive(joint_state_);

        // Setup some state vectors
        Eigen::VectorXd q = Eigen::VectorXd::Zero(kNumTotalDofs);
        Eigen::VectorXd q_dot = Eigen::VectorXd::Zero(kNumTotalDofs);
        Eigen::VectorXd tau = Eigen::VectorXd::Zero(kNumTotalDofs);

        // Get orientation Quaternion
        Eigen::Quaterniond orientation = Eigen::Map<Eigen::Quaterniond>(com_state_.orientation);
        Eigen::Matrix3d R_b = orientation.toRotationMatrix();


        // Get Root Joint
        dart::dynamics::Joint *root = robot_->getRootJoint();
        
        auto floatingJoint = dynamic_cast<dart::dynamics::FreeJoint *>(root);

        floatingJoint->setTransform()
        //dart::dynamics::Joint *dtJoint = robot_>getRootJoint();

        // A free (root) joint won't be in the Gazebo model, so handle it seperately.
//         auto floatingBase = 
//   auto dtFreeJoint = dynamic_cast<dart::dynamics::FreeJoint *>(dtJoint);
//   if (dtFreeJoint != nullptr)
//   {
//     // Set skeleton pose
//     dtFreeJoint->setTransform(
//         physics::DARTTypes::ConvPose(this->dataPtr->model->WorldPose()));
//     // Get model velocity
//     ignition::math::Vector3d linVel = this->dataPtr->model->WorldLinearVel();
//     ignition::math::Vector3d angVel = this->dataPtr->model->WorldAngularVel();
//     // Set skeleton velocity
//     dtFreeJoint->setLinearVelocity(
//         Eigen::Vector3d(linVel.X(), linVel.Y(), linVel.Z()));
//     dtFreeJoint->setAngularVelocity(
//         Eigen::Vector3d(angVel.X(), angVel.Y(), angVel.Z()));



        // TODO: Function/Wrap This to set floating base state
        Eigen::Isometry3d tf;
        tf.linear() = R_b;
        tf.translation() = Eigen::Map<Eigen::VectorXd>(com_state_.pos_world, 3);

        // Use special helper to set orientation. Could be a better way to do this.  But it doesn't seem you
        // Angles are exponentially mapped so direct RPY is not supported
        Eigen::VectorXd floating_pos = dart::dynamics::FreeJoint::convertToPositions(tf);
        
        // Copy into our state vectors
        q.head(kNumFloatingDofs) = floating_pos;
        q.tail(kNumActuatedDofs) = Eigen::Map<Eigen::VectorXd>(joint_state_.q, kNumActuatedDofs);

        // TODO: Need to verify this.  These values need to be in body coordinates.  Can't tell an actual difference in the control?
        q_dot.segment(0, 3) = /* R_b.transpose() * */Eigen::Map<Eigen::VectorXd>(com_state_.omega_world, 3);
        q_dot.segment(3, 3) = /* R_b.transpose() * */Eigen::Map<Eigen::VectorXd>(com_state_.vel_world, 3);
        q_dot.tail(kNumActuatedDofs) = Eigen::Map<Eigen::VectorXd>(joint_state_.q_dot, kNumActuatedDofs);

        // Update Joint Torques
        tau.tail(kNumActuatedDofs) = Eigen::Map<Eigen::VectorXd>(joint_state_.tau, kNumActuatedDofs);

        // Copy to internal dynamics model
        robot_->setPositions(q);
        robot_->setVelocities(q_dot);
        robot_->setForces(tau);

        // Update Dynamics State
        robot_->computeForwardKinematics();
        robot_->computeForwardDynamics();

        // Get Updated q state (we need to modify it for the floating base orientation)
        Eigen::VectorXd q_new = robot_->getPositions();

         // Get Euler Angles from Orientation Quaternion
        q_new.head(3) = Common::Math::QuaterionToEuler(orientation);

        // Update Velcity Frames
        q_dot.segment(0, 3) = Eigen::Map<Eigen::VectorXd>(com_state_.omega_world, 3);
        q_dot.segment(3, 3) = Eigen::Map<Eigen::VectorXd>(com_state_.vel_world, 3);

        // Copy Data over for our Full Robot State Message
        Eigen::Map<Eigen::VectorXd>(full_state_.q, kNumTotalDofs) = q_new;
        Eigen::Map<Eigen::VectorXd>(full_state_.q_dot, kNumTotalDofs) = robot_->getVelocities();
        Eigen::Map<Eigen::MatrixXd>(full_state_.M, kNumTotalDofs, kNumTotalDofs) = robot_->getMassMatrix();
        Eigen::Map<Eigen::VectorXd>(full_state_.b, kNumTotalDofs) = robot_->getCoriolisForces();
        Eigen::Map<Eigen::VectorXd>(full_state_.g, kNumTotalDofs) = robot_->getGravityForces();

        // Compute Foot Positions
        double x_sum = 0.0;
        double y_sum = 0.0;
        for (int i = 0; i < NUM_LEGS; i++)
        {
            // Foot Position
            Eigen::Map<Eigen::Vector3d>(&full_state_.foot_pos[i * 3], 3) = foot_body_[i]->getTransform(hip_base_body_[i]).translation();

            // Foot Position World
            Eigen::Vector3d foot_pos_world = foot_body_[i]->getTransform(dart::dynamics::Frame::World()).translation();
            Eigen::Map<Eigen::Vector3d>(&full_state_.foot_pos_wcs[i * 3], 3) = foot_pos_world;

            // TODO: Will eventually need to be projected to estimated ground beneath robot
            x_sum += foot_pos_world.x();
            y_sum += foot_pos_world.y();

            // Update Augmented Contact Jacobian
            J_legs_.block<3, kNumTotalDofs>(i*3, 0) = robot_->getLinearJacobian(foot_body_[i], hip_base_body_[i], base_body_);
        }

        // Compute Center of Support
        // TODO: Need Contact Schedule to compute.  TBH this is going to need to be somewhere else.
        Eigen::Vector3d CoS_world;
        CoS_world.x() = x_sum/kNumContacts;
        CoS_world.y() = y_sum/kNumContacts;
        CoS_world.z() = 0.0;

        Eigen::Map<Eigen::Vector3d>(full_state_.CoS_wcs) = CoS_world;
        Eigen::Map<Eigen::Vector3d>(full_state_.CoS) = R_b.transpose() * CoS_world;

        // Update Jacobian for our Full Robot State Message
        Eigen::Map<Eigen::MatrixXd>(full_state_.J_c, 3 * kNumContacts, kNumTotalDofs) = J_legs_;

        // Now Update Foot Velocities
        Eigen::Map<Eigen::VectorXd>(full_state_.foot_vel, kNumActuatedDofs) = (J_legs_ * robot_->getVelocities());
        //Eigen::Map<Eigen::VectorXd>(full_state_.foot_vel, kNumActuatedDofs) = (J_legs_.rightCols(kNumActuatedDofs) * robot_->getVelocities().tail(kNumActuatedDofs));

        // Publish Leg Command
        GetOutputPort(OutputPort::FULL_STATE)->Send(full_state_);
    }

    // Update function for next state from inputs
    void NomadDynamics::UpdateState()
    {

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
