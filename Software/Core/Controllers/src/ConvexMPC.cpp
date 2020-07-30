/*
 * ConvexMPC.cpp
 *
 *  Created on: July 13, 2019
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
#include <Controllers/ConvexMPC.hpp>

// C System Includes

// C++ System Includes
#include <iostream>
#include <string>

// Third-Party Includes

// Project Includes
#include <Realtime/RealTimeTask.hpp>

namespace Controllers
{

namespace Locomotion
{
//using namespace RealTimeControl;
ConvexMPC::ConvexMPC(const std::string &name, const unsigned int N, const double T) : Realtime::RealTimeTaskNode(name, 20000, Realtime::Priority::MEDIUM, -1, PTHREAD_STACK_MIN),
                                                                                      num_states_(13),
                                                                                      num_inputs_(1),
                                                                                      T_(T),
                                                                                      N_(N)
{
    // Sample Time
    T_s_ = T_ / (N_);

    // Create OCP
    ocp_ = std::make_unique<OptimalControl::LinearOptimalControl::LinearCondensedOCP>(N_, T_, 2, 1, false);

    // TODO: Should be SET from outside
    // Create Rigid Body
    block_ = RigidBlock1D(1.0, Eigen::Vector3d(1.0, 0.5, 0.25), T_s_);
    //std::cout << "BLOCK: " << T_s_ << std::endl;
    // State Weights
    Eigen::VectorXd Q(2);
    Q[0] = 100.0;
    Q[1] = 1.0;

    // Input Weights
    Eigen::VectorXd R(1);
    R[0] = 0.1;

    ocp_->SetWeights(Q, R);

    // Create Messages
    force_output_.length = num_inputs_;
    force_output_.data.resize(num_inputs_);

    // TODO: Move to "CONNECT"
    // Create Ports
    // State Estimate Input Port
    input_port_map_[InputPort::STATE_HAT] = std::make_shared<Realtime::Port>("STATE_HAT", Realtime::Port::Direction::INPUT, Realtime::Port::DataType::DOUBLE, num_states_, rt_period_);

    // Referenence Input Port
    input_port_map_[InputPort::REFERENCE_TRAJECTORY] = std::make_shared<Realtime::Port>("REFERENCE", Realtime::Port::Direction::INPUT, Realtime::Port::DataType::DOUBLE, num_states_, rt_period_);

    // Optimal Force Solution Output Port
    output_port_map_[OutputPort::FORCES] = std::make_shared<Realtime::Port>("FORCES", Realtime::Port::Direction::OUTPUT, Realtime::Port::DataType::DOUBLE, num_inputs_, rt_period_);
}
void ConvexMPC::Run()
{
    static int i = 0;
    // Get Inputs
    // std::cout << "Time to RECEIVE in CONVEXMPC" << std::endl;
    // Receive State Estimate and Unpack
    bool state_recv = GetInputPort(InputPort::STATE_HAT)->Receive(x_hat_in_); // Receive State Estimate

    // Receive Trajectory Reference and Unpack
    bool setpoint_recv = GetInputPort(InputPort::REFERENCE_TRAJECTORY)->Receive(reference_in_); // Receive Setpoint
    if (!state_recv || !setpoint_recv)
    {
       // std::cout << "[ConvexMPC]: Receive Buffer Empty!" << std::endl;
        return;
    }
    // std::cout << "CMPC: " << x_hat_in_.sequence_num;

   // std::cout << "SIZE: " << reference_in_.length << std::endl;
   // std::cout << "SIZE: " << num_states_*N_ << std::endl;

    Eigen::VectorXd x_hat_ = Eigen::Map<Eigen::VectorXd>(x_hat_in_.data.data(), num_states_);
    Eigen::MatrixXd X_ref_ = Eigen::Map<Eigen::MatrixXd>(reference_in_.data.data(), num_states_, N_);
     //std::cout <<  X_ref_ << std::endl;
     //std::cout <<  x_hat_ << std::endl;

    // Update our Dynamics Current State
    Eigen::VectorXd initial_state(2);
    initial_state[0] = x_hat_[0];
    initial_state[1] = x_hat_[3];

    //std::cout << i++ <<std::endl;
    //std::cout << initial_state << std::endl;
    block_.SetState(initial_state);

    // Pass to Optimal Control Problem
    ocp_->SetInitialCondition(block_.GetState());
    ocp_->SetModelMatrices(block_.A_d(), block_.B_d());
    Eigen::MatrixXd ref_test(2, N_);
    ref_test.row(0) = X_ref_.row(0);
    ref_test.row(1) = X_ref_.row(3);
   // std::cout << "Refactor: " << std::endl;
    //std::cout <<  initial_state << std::endl;
   // std::cout <<  ref_test << std::endl;

    ocp_->SetReference(ref_test);
    

    // Solve
    ocp_->Solve();

    //std::cout <<  "SOLVED: " << std::endl;

    // for (int i = 0; i < N_ - 1; i++)
    // {
         //std::cout << "U: " << ocp_->U() << std::endl;
      //   block_.Step(ocp_->U()(0, i));
       //  std::cout << "X: " << block_.GetState() << std::endl;
   //s  }

    force_output_.data[0] = ocp_->U()(0, 0);

    // Output Optimal Forces
    bool send_status = GetOutputPort(OutputPort::FORCES)->Send(force_output_);
}
void ConvexMPC::Setup()
{
    // Connect Input Ports
    //GetInputPort(InputPort::STATE_HAT)->Connect();            // State Estimate
    //GetInputPort(InputPort::REFERENCE_TRAJECTORY)->Connect(); // Reference Trajectory

    // Bind Output Ports
    GetOutputPort(OutputPort::FORCES)->Bind(); // Optimal Force Output

    std::cout << "[ConvexMPC]: "
              << "ConvexMPC Task Node Running!" << std::endl;
}

} // namespace Locomotion
} // namespace Controllers
