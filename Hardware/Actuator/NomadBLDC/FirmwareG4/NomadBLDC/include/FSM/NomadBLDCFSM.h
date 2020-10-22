/*
 * NomadBLDCFSM.h
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

#ifndef NOMADBLDC_FSM_NOMADBLDCFSM_H_
#define NOMADBLDC_FSM_NOMADBLDCFSM_H_

// C System Files

// C++ System Files

// Third Party Includes

// Project Include Files
#include <FSM/FiniteStateMachine.h>
#include <FSM/NomadBLDCData.h>

#include "LEDService.h"

// Finite State Machine Class
class NomadBLDCFSM : public FiniteStateMachine
{
    friend class NomadState;

public:

    // typedef enum
    // {
    //     OFF = 0,
    //     IDLE = 1,
    //     ESTOP = 7
    // } CONTROL_MODE;

    // Base Class Primary Controller FSM
    NomadBLDCFSM();

    // Run an iteration of the state machine
    bool Run(float dt);

    // Return Mode from Active State
    NomadBLDCData* GetData() const;

protected:
    // Helper function to create state machine
    void _CreateFSM();

    // Shared Data Pointer.  TODO: Switch to ZCM Port?
    NomadBLDCData *data_;
};

class NomadBLDCTransitionEvent : public TransitionEvent
{
public:
    NomadBLDCTransitionEvent(const std::string &name, NomadBLDCData *data);

protected:
    NomadBLDCData *data_;
    std::string name_;
};

// Transitions
class CommandModeEvent : public NomadBLDCTransitionEvent
{
public:
    // Base Class Transition Event
    // name = Transition Event name
    CommandModeEvent(const std::string &name,
                     control_mode_type_t mode,
                     NomadBLDCData *data) : NomadBLDCTransitionEvent(name, data), req_mode_(mode)
    {
    }

    // Stop state machine and cleans up
    bool Triggered()
    {
        //std::cout << "Check: " << data_->control_mode << std::endl;
        if (data_->controller->GetControlMode() == req_mode_)
        {
            //std::cout << "Event ID: " << name_ << " is SET!" << std::endl;
            return true;
        }
        return false;
    };

protected:
    control_mode_type_t req_mode_;
};
#endif // NOMADBLDC_FSM_NOMADBLDCFSM_H_
