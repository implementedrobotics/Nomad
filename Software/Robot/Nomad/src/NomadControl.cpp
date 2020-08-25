/*
 * NomadControl.cpp
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

// Primary Include
#include <Nomad/NomadControl.hpp>

// C System Includes

// C++ System Includes
#include <iostream>
#include <string>
#include <sstream>
#include <memory>

// Third-Party Includes

// Project Includes
#include <Controllers/LegController.hpp>
#include <Communications/Messages/double_vec_t.hpp>
#include <Communications/Messages/generic_msg_t.hpp>
#include <Common/Time.hpp>

namespace Robot::Nomad::Controllers
{
    NomadControl::NomadControl(const double T_s)  : SystemBlock("FSM_Controller", T_s)
    {
        // Create Ports
        // Primary Controller Input Port
        input_port_map_[InputPort::TELEOP_DATA] = Communications::Port::CreateInput<teleop_data_t>("TELEOP_DATA");
        input_port_map_[InputPort::FULL_STATE] = Communications::Port::CreateInput<full_state_t>("FULL_STATE");

        // Primary Controller Output Ports
        output_port_map_[OutputPort::LEG_COMMAND] = Communications::Port::CreateOutput("LEG_COMMAND");

        // Create FSM
        nomad_control_FSM_ = std::make_shared<Robot::Nomad::FSM::NomadControlFSM>(this);
    }

    void NomadControl::UpdateStateOutputs()
    {
        // Receive Data
    }

    // Update function for stateless outputs
    void NomadControl::UpdateStatelessOutputs()
    {
        // Read teleop data
        GetInputPort(InputPort::TELEOP_DATA)->Receive(teleop_data_);

        // Update Data
        // TODO: Map/Forward port into FSM
        nomad_control_FSM_->GetData()->control_mode = teleop_data_.control_mode;
        nomad_control_FSM_->GetData()->theta = teleop_data_.theta;
        nomad_control_FSM_->GetData()->phi = teleop_data_.phi;
        //nomad_control_FSM_->GetData()->nomad_state = full_state_;

        // Run FSM
        nomad_control_FSM_->Run(T_s_);

    }

    // Update function for next state from inputs
    void NomadControl::UpdateState()
    {

    }

    void NomadControl::Setup()
    {
        // Run Base Setup Routine
        SystemBlock::Setup();

        // Start FSM
        nomad_control_FSM_->Start(Systems::Time::GetTime<double>());

        std::cout << "[NomadControl]: "
                  << "Nomad Control FSM Publisher Running!: " << std::endl;
    }
} // namespace Robot::Nomad::Controllers
