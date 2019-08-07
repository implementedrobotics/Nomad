/*
 * NomadPlant.cpp
 *
 *  Created on: August 5, 2019
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
#include <Systems/NomadPlant.hpp>

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


namespace Systems
{
namespace Nomad
{

NomadPlant::NomadPlant(const std::string &name) : 
                               Realtime::RealTimeTaskNode(name, 20000, Realtime::Priority::MEDIUM, -1, PTHREAD_STACK_MIN)
{

    // TODO: Should be SET from outside
    // Create Rigid Body
    block_ = RigidBlock1D(2.0, Eigen::Vector3d(1.0, 0.5, 0.25));

    // TODO: Move to "CONNECT"
    // Create Ports
    // State Estimate Input Port
    input_port_map_[InputPort::FORCES] = std::make_shared<Realtime::Port>("FORCES", Realtime::Port::Direction::INPUT, Realtime::Port::DataType::DOUBLE, 1, rt_period_);

    // Optimal Force Solution Output Port
    //output_port_map_[OutputPort::FORCES] = std::make_shared<Realtime::Port>("FORCES", Realtime::Port::Direction::OUTPUT, Realtime::Port::DataType::DOUBLE, num_inputs_, rt_period_);

}

void NomadPlant::Run()
{
     // Get Inputs
    // std::cout << "Time to RECEIVE in CONVEXMPC" << std::endl;
    // Receive State Estimate and Unpack
    bool force_recv = GetInputPort(InputPort::FORCES)->Receive(forces_in); // Receive State Estimate

    if (!force_recv)
    {
        std::cout << "[NomadPlant]: Receive Buffer Empty!" << std::endl;
        return;
    }
    // std::cout << "CMPC: " << x_hat_in_.sequence_num;
    Eigen::VectorXd U = Eigen::Map<Eigen::VectorXd>(forces_in.data.data(), 1);
    block_.Step(U);

    std::cout << "U: " << U << std::endl;

    // TODO: Output back state

}

void NomadPlant::Setup()
{
    // Connect Input Ports
    GetInputPort(InputPort::FORCES)->Connect(); // Forces

    // Bind Output Ports
    //GetOutputPort(OutputPort::FORCES)->Bind(); // Optimal Force Output

    std::cout << "[NomadPlant]: "
              << "NomadPlant Task Node Running!" << std::endl;
}

} // namespace Nomad
} // namespace Systems
