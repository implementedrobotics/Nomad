/*
 * StandState.hpp
 *
 *  Created on: July 1, 2020
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

#ifndef NOMAD_CONTROL_STANDSTATE_H_
#define NOMAD_CONTROL_STANDSTATE_H_

// C System Files

// C++ System Files

// Third Party Includes

// Project Include Files
#include <Nomad/FSM/NomadState.hpp>
#include <Nomad/NomadRobot.hpp>
#include <Common/Math/CubicPolynomialTrajectory.hpp>


namespace Robot::Nomad::FSM
{
    class StandState : public NomadState
    {

    public:
        StandState();

        // Called upon a state change and we enter this state
        // current_time = current robot/controller time
        void Enter_(double current_time);

        // // current_time = current robot/controller time
        // // Called upon a state change and we are exiting this state
        // void Exit(double current_time);

        // Logic to run each iteration of the state machine run
        // dt = time step for this iteration
        void Run_(double dt);

    private:
        full_state_t nomad_state_initial_;
        Common::CubicPolynomialTrajectory stand_traj_[Robot::Nomad::NUM_LEGS];
        Common::CubicPolynomialTrajectory com_traj_;

    };
} // namespace Robot::Nomad::FSM
#endif // NOMAD_CONTROL_STANDSTATE_H_