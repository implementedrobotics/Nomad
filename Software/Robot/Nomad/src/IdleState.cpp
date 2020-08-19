
/*
 * IdleState.cpp
 *
 *  Created on: June 27, 2020
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

// C System Files

// C++ System Files
#include <iostream>

// Third Party Includes

// Project Include Files
#include <Nomad/FSM/IdleState.hpp>
#include <Nomad/NomadControl.hpp>
#include <Controllers/Messages/leg_controller_cmd_t.hpp>

using Robot::Nomad::Controllers::NomadControl;

namespace Robot::Nomad::FSM
{
    IdleState::IdleState() : NomadState("IDLE", 1)
    {
    }
    void IdleState::Run_(double dt)
    {
       // std::cout << "Idle Running" << std::endl;

        // Set mode to idle

        // Zero out leg command
        leg_controller_cmd_t leg_command;
        memset(&leg_command, 0, sizeof(leg_controller_cmd_t));

        // Output Leg Command
        GetOutputPort(NomadControl::OutputPort::LEG_COMMAND)->Send(leg_command);

    }
    void IdleState::Enter_(double current_time)
    {
        std::cout << "Entering Idle State!!!" << std::endl;
        // current_mode_ = ControlMode::OFF;
        // Zero out leg command
        leg_controller_cmd_t leg_command;
        memset(&leg_command, 0, sizeof(leg_controller_cmd_t));

        Eigen::Map<Eigen::VectorXd>(leg_command.k_p_cartesian, 12) = Eigen::VectorXd::Zero(12);
        Eigen::Map<Eigen::VectorXd>(leg_command.k_d_cartesian, 12) = Eigen::VectorXd::Ones(12) * 25;

        // Output Leg Command
        GetOutputPort(NomadControl::OutputPort::LEG_COMMAND)->Send(leg_command);

    }
} // namespace Robot::Nomad::FSM
