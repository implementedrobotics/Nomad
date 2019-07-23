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
ConvexMPC::ConvexMPC(const std::string &name, const unsigned int N, const double T) : 
                               Realtime::RealTimeTaskNode(name, 20000, Realtime::Priority::MEDIUM, -1, PTHREAD_STACK_MIN),
                               sequence_num_(0),
                               num_states_(13),
                               num_inputs_(12),
                               T_(T),
                               N_(N)
{
    // Sample Time
    T_s_ = T_ / (N_);

    // Create OCP
    ocp_ = new OptimalControl::LinearOptimalControl::LinearCondensedOCP(N_, T_, 2,1,false);

    // TODO: Should be SET from outside
    // Create Rigid Body
    block_ = RigidBlock1D(2.0, Eigen::Vector3d(1.0, 0.5, 0.25));

    // State Weights
    Eigen::VectorXd Q(2);
    Q[0] = 100.0;
    Q[1] = 1.0;

    // Input Weights
    Eigen::VectorXd R(1);
    R[0] = 0.1;
    
    ocp_->SetWeights(Q, R);

    // Create Ports
    zmq::context_t *ctx = Realtime::RealTimeTaskManager::Instance()->GetZMQContext();

    // State Estimate Input Port
    // TODO: Independent port speeds.  For now all ports will be same speed as task node
    Realtime::Port *port = new Realtime::Port("STATE_HAT", ctx, "state", rt_period_);
    input_port_map_[InputPort::STATE_HAT] = port;

    // Referenence Input Port
    port = new Realtime::Port("REFERENCE", ctx, "reference", rt_period_);
    input_port_map_[InputPort::REFERENCE_TRAJECTORY] = port;

    // Optimal Force Solution Output Port
    port = new Realtime::Port("FORCES", ctx, "forces", rt_period_);
    output_port_map_[OutputPort::FORCES] = port;


}
void ConvexMPC::Run()
{  
     // Get Inputs

    // Receive State Estimate and Unpack
    bool state_recv = GetInputPort(0)->Receive((void *)&x_hat_in_, sizeof(x_hat_in_)); // Receive State Estimate

    // Receive Trajectory Reference and Unpack
    bool setpoint_recv =  GetInputPort(1)->Receive((void *)&reference_in_, sizeof(reference_in_)); // Receive Setpoint
    if(!state_recv || !setpoint_recv)
    {
        std::cout << "[ConvexMPC]: Receive Buffer Empty!" << std::endl; 
        return;
    }

    // Get Timestamp
    // TODO: "GetUptime" Static function in a time class
    uint64_t time_now = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();

    Eigen::VectorXd x_hat_ = Eigen::Map<Eigen::VectorXd>(x_hat_in_.x, 13);
    // Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> X_ref_ = Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>>(reference_in_.X_ref,13,24);
    Eigen::MatrixXd X_ref_ = Eigen::Map<Eigen::MatrixXd>(reference_in_.X_ref,13,10);
    //std::cout <<  X_ref_ << std::endl;
    //std::cout <<  x_hat_ << std::endl;

    // Update our Dynamics Current State
    Eigen::VectorXd initial_state(2);
    initial_state[0] = x_hat_[0];
    initial_state[1] = x_hat_[1];

    block_.SetState(initial_state);

    // Pass to Optimal Control Problem
    ocp_->SetInitialCondition(block_.GetState());
    ocp_->SetModelMatrices(block_.A_d(), block_.B_d());
    Eigen::MatrixXd ref_test(2,N_);
    ref_test.row(0) = X_ref_.row(0);
    ref_test.row(1) = X_ref_.row(3);
    //std::cout << "Refactor: " << std::endl;
    std::cout <<  initial_state << std::endl;
    std::cout <<  ref_test << std::endl;

    ocp_->SetReference(ref_test);

    // Solve
    ocp_->Solve();

    for(int i = 0; i < N_-1; i++)
    {
        //std::cout << "U: " << ocp_->U() << std::endl;
        block_.Step(ocp_->U()(0,i));
        //std::cout << "X: " << block_.GetState()[0] << std::endl;
    }

    // Output Optimal Forces
    GetOutputPort(0)->Send((void *)&force_output_, sizeof(force_output_));
    // GetOutputPort(0)->Send(rx_msg);
    sequence_num_++;
}
void ConvexMPC::Setup()
{
    // Connect Input Ports
    GetInputPort(0)->Connect(); // State Estimate
    GetInputPort(1)->Connect(); // Reference Trajectory

    // Bind Output Ports
    GetOutputPort(0)->Bind(); // Optimal Force Output

    std::cout << "[ConvexMPC]: "
              << "ConvexMPC Task Node Running!" << std::endl;
}

} // namespace Locomotion
} // namespace Controllers
