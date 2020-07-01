/*
 * Gamepad.cpp
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
#include <OperatorInterface/GamepadTeleopFSM/States/GamepadState.hpp>

namespace OperatorInterface
{
    namespace Teleop
    {

        OffState::OffState() : GamepadState("OFF", 0)
        {
        }
        void OffState::Run(double dt)
        {
            //std::cout << "Off Running" << std::endl;
            // Set mode to idle
        }
        void OffState::Enter(double current_time)
        {
            std::cout << "Entering Off State" << std::endl;
            current_mode_ = ControlMode::OFF;
        }



        IdleState::IdleState() : GamepadState("IDLE", 1)
        {
        }
        void IdleState::Run(double dt)
        {
            //std::cout << "Idle Running" << std::endl;
            // Set mode to idle
        }
        void IdleState::Enter(double current_time)
        {
            std::cout << "Entering Idle State" << std::endl;
            current_mode_ = ControlMode::IDLE;
        }


        StandState::StandState() : GamepadState("STAND", 2)
        {
        }
        void StandState::Run(double dt)
        {
            //std::cout << "Stand Running" << std::endl;
            // Set mode to stand
        }
        void StandState::Enter(double current_time)
        {
            std::cout << "Entering Stand State" << std::endl;
            current_mode_ = ControlMode::STAND;
        }

        SitState::SitState() : GamepadState("SIT", 3)
        {
        }
        void SitState::Run(double dt)
        {
            //std::cout << "Sit Running" << std::endl;
            // Set mode to stand
        }
        void SitState::Enter(double current_time)
        {
            std::cout << "Entering Sit State" << std::endl;
            current_mode_ = ControlMode::SIT;
        }


    } // namespace Teleop
} // namespace OperatorInterface