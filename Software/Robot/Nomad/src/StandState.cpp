
/*
 * StandState.cpp
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

#include <Nomad/FSM/StandState.hpp>

namespace Robot::Nomad::FSM
{
    StandState::StandState() : NomadState("STAND", 2)
    {
    }
    void StandState::Run_(double dt)
    {
       std::cout << "Stand Running: " << elapsed_time_ << std::endl;
       full_state_t nomad_state_ = data_->nomad_state;
    }
    void StandState::Enter_(double current_time)
    {
        std::cout << "Entering Stand State!!! | " << current_time << std::endl;
        nomad_state_initial_ = data_->nomad_state;

        // Create Cubic Trajectory
        double stand_height = .35; // TODO: From Parameter/ControlData
        // TODO: 1.0 = Stand Time, Make Param
        stand_traj_[Robot::Nomad::FRONT_LEFT].Generate(nomad_state_initial_.foot_pos[Robot::Nomad::FOOT_FL_Z], -stand_height, 0.0, 0.0, 0.0, 1.0);
        stand_traj_[Robot::Nomad::FRONT_RIGHT].Generate(nomad_state_initial_.foot_pos[Robot::Nomad::FOOT_FR_Z], -stand_height, 0.0, 0.0, 0.0, 1.0);
        stand_traj_[Robot::Nomad::REAR_LEFT].Generate(nomad_state_initial_.foot_pos[Robot::Nomad::FOOT_RL_Z], -stand_height, 0.0, 0.0, 0.0, 1.0);
        stand_traj_[Robot::Nomad::REAR_RIGHT].Generate(nomad_state_initial_.foot_pos[Robot::Nomad::FOOT_RR_Z], -stand_height, 0.0, 0.0, 0.0, 1.0);
    }
} // namespace Robot::Nomad::FSM
