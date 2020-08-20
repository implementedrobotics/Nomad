/*
 * NomadControl.hpp
 *
 *  Created on: June 30, 2020
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

#ifndef ROBOT_NOMAD_NOMADCONTROL_H_
#define ROBOT_NOMAD_NOMADCONTROL_H_

// C System Files
#include <stdint.h>

// C++ System Files
#include <iostream>
#include <string>

// Third Party Includes
#include <Eigen/Dense>
#include <Communications/Messages/double_vec_t.hpp>
#include <Communications/Messages/int32_vec_t.hpp>
#include <Communications/Messages/generic_msg_t.hpp>
#include <Nomad/MessageTypes/com_state_t.hpp>
#include <Nomad/MessageTypes/joint_state_t.hpp>
#include <Nomad/MessageTypes/full_state_t.hpp>
#include <Nomad/MessageTypes/teleop_data_t.hpp>

// Project Includes
#include <Systems/SystemBlock.hpp>
#include <Controllers/LegController.hpp>
#include <Nomad/FSM/NomadControlFSM.hpp>

namespace Robot::Nomad::Controllers
{
    class NomadControl : public Core::Systems::SystemBlock
    {

    public:
        enum OutputPort
        {
            LEG_COMMAND = 0,
            NUM_OUTPUTS = 1
        };

        enum InputPort
        {
            TELEOP_DATA = 0,
            FULL_STATE = 1,
            NUM_INPUTS = 2
        };

        // Base Class Nomad Control Task Node
        // T_s = System sampling time
        NomadControl(const double T_s = -1);

    protected:

        // Do any pre run setup here
        virtual void Setup();

        // Update function for stateful outputs
        void UpdateStateOutputs();

        // Update function for stateless outputs
        void UpdateStatelessOutputs();

        // Update fucntion for next state from inputs
        void UpdateState();

        // (Input) Full Robot State
        full_state_t full_state_;

        // (Input) Teleop Command Data
        teleop_data_t teleop_data_;

        // Output
        //generic_msg_t leg_command_msg_;
        //::Controllers::Locomotion::leg_controller_cmd_t leg_controller_cmd_;

        // FSM
        std::shared_ptr<Robot::Nomad::FSM::NomadControlFSM> nomad_control_FSM_;
    };
} // namespace Robot::Nomad::Controllers

#endif // ROBOT_NOMAD_NOMADCONTROL_H_
