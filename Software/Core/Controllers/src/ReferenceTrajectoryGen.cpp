/*
 * ReferenceTrajectoryGenerator.cpp
 *
 *  Created on: July 16, 2019
 *      Author: Quincy Jones
 *
 * Copyright (c) <2019> <Quincy Jones - quincy@implementedrobotics.com/>
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
#include <Controllers/ReferenceTrajectoryGen.hpp>

// C System Includes

// C++ System Includes
#include <iostream>
#include <string>
#include <sstream>
#include <chrono>

// Third-Party Includes
#include <Eigen/Dense>

// Project Includes
#include <Realtime/RealTimeTask.hpp>

// TODO: Static Variable in "Physics" Class somewhere

double kGravity = 9.81;

namespace Controllers::Locomotion
{
    //using namespace RealTimeControl;

    ReferenceTrajectoryGenerator::ReferenceTrajectoryGenerator(const std::string &name, const unsigned int N, const double T) : Realtime::RealTimeTaskNode(name, 20000, Realtime::Priority::MEDIUM, -1, PTHREAD_STACK_MIN),
                                                                                                                                num_states_(13),
                                                                                                                                T_(T),
                                                                                                                                N_(N)
    {

        // Sample Time
        T_s_ = T_ / (N_);

        // Reference State Trajectory Holder
        X_ref_ = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>(num_states_, N_);

        // Create Messages
        reference_out_.length = X_ref_.size();
        reference_out_.data.resize(reference_out_.length);

        // Create Ports
        // Reference Output Port
        // TODO: Independent port speeds.  For now all ports will be same speed as task node
        output_port_map_[OutputPort::REFERENCE] = std::make_shared<Communications::Port>("REFERENCE", Communications::Port::Direction::OUTPUT, Communications::Port::DataType::DOUBLE, num_states_, rt_period_);

        // TODO: Move to "CONNECT"
        input_port_map_[InputPort::STATE_HAT] = std::make_shared<Communications::Port>("STATE_HAT", Communications::Port::Direction::INPUT, Communications::Port::DataType::DOUBLE, num_states_, rt_period_);

        input_port_map_[InputPort::SETPOINT] = std::make_shared<Communications::Port>("SETPOINT", Communications::Port::Direction::INPUT, Communications::Port::DataType::DOUBLE, 4, rt_period_);
    }

    void ReferenceTrajectoryGenerator::Run()
    {
        // Get Inputs
        bool state_recv = GetInputPort(InputPort::STATE_HAT)->Receive(x_hat_in_);      // Receive State Estimate
        bool setpoint_recv = GetInputPort(InputPort::SETPOINT)->Receive(setpoint_in_); // Receive Setpoint

        // TODO: Add a metric for how far behind this node can get before erroring out.
        if (!state_recv || !setpoint_recv)
        {
            //std::cout << "[ReferenceTrajectoryGenerator]: Receive Buffer Empty!" << std::endl;
            return;
        }

        // Get Timestamp
        // TODO: "GetUptime" Static function in a time class
        uint64_t time_now = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
        Eigen::VectorXd x_hat_ = Eigen::Map<Eigen::VectorXd>(x_hat_in_.data.data(), 13);

        double x_dot = setpoint_in_.data[0];
        double y_dot = setpoint_in_.data[1];
        double yaw_dot = setpoint_in_.data[2];
        double z_com = setpoint_in_.data[3];

        // Compute Trajectory
        X_ref_(0, 0) = x_hat_in_.data[0]; // X Position
        X_ref_(1, 0) = x_hat_in_.data[1]; // Y Position
        X_ref_.row(2).setConstant(z_com); // Z Position

        X_ref_.row(3).setConstant(x_dot); // X Velocity
        X_ref_.row(4).setConstant(y_dot); // Y Velocity
        X_ref_.row(5).setConstant(0);     // Z Velocity

        X_ref_.row(6).setConstant(0);                 // Roll Orientation
        X_ref_.row(7).setConstant(0);                 // Pitch Orientation
        X_ref_.row(8).setConstant(x_hat_in_.data[8]); // Yaw Orientation

        X_ref_.row(9).setConstant(0);  // Roll Rate
        X_ref_.row(10).setConstant(0); // Pitch Rate

        // Since we linearize and assume pitch and roll are 0.  omega_z = yaw_rate
        //Eigen::Vector3d orientation_dot(0,0,setpoint.yaw_dot);
        //Eigen::Matrix3d R_z = Eigen::AngleAxisd(x_hat.x[8], Eigen::Vector3d(0,0,1)).toRotationMatrix();
        //Eigen::Vector3d omega = R_z * orientation_dot;
        //std::cout << orientation_dot << std::endl;
        //std::cout << R_z << std::endl;
        //std::cout << omega;

        X_ref_.row(11).setConstant(yaw_dot);  // Yaw Rate
        X_ref_.row(12).setConstant(kGravity); // Gravity

        for (int i = 0; i < N_ - 1; i++)
        {
            X_ref_(0, i + 1) = X_ref_(0, i) + x_dot * T_s_;
            X_ref_(1, i + 1) = X_ref_(1, i) + y_dot * T_s_;
            X_ref_(8, i + 1) = X_ref_(8, i) + yaw_dot * T_s_;
        }
        //std::cout << "XREF: " << X_ref_ << std::endl;

        // Update Publish Trajectory Buffer
        memcpy(reference_out_.data.data(), X_ref_.data(), sizeof(double) * X_ref_.size());

        // Publish Trajectory
        bool send_status = GetOutputPort(OutputPort::REFERENCE)->Send(reference_out_);
    }

    void ReferenceTrajectoryGenerator::Setup()
    {

        // State Estimate INPUT
        //GetInputPort(InputPort::STATE_HAT)->Connect();

        // Setpoint INPUT
        //GetInputPort(InputPort::SETPOINT)->Connect();

        // Reference OUTPUT
        GetOutputPort(OutputPort::REFERENCE)->Bind();
    }

} // namespace Controllers::Locomotion
