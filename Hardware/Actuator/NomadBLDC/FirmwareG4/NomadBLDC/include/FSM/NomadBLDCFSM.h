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
#include <LEDService.h>
typedef enum
{
    STATE_STARTUP = 0,
    STATE_IDLE = 1,
    STATE_ERROR = 2,
    STATE_FOC = 3,
    STATE_CALIB_RESISTANCE = 4,
    STATE_CALIB_INDUCTANCE = 5,
    STATE_CALIB_PHASE_ORDER = 6,
    STATE_CALIB_ENCODER = 7
} NomadBLDCStateID;

// Finite State Machine Class
class NomadBLDCFSM : public FiniteStateMachine
{
    friend class NomadState;

public:

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
    NomadBLDCTransitionEvent(/*const std::string &name, */NomadBLDCData *data);

protected:
    NomadBLDCData *data_;
};

// Transitions
class CommandModeEvent : public NomadBLDCTransitionEvent
{
public:
    // Base Class Transition Event
    // name = Transition Event name
    CommandModeEvent(/*const std::string &name,*/
                     control_mode_type_t mode,
                     NomadBLDCData *data) : NomadBLDCTransitionEvent(data), req_mode_(mode)
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

// Transitions
class FaultModeEvent : public NomadBLDCTransitionEvent
{
public:
    // Base Class Transition Event
    // name = Transition Event name
    FaultModeEvent(/*const std::string &name,*/
                     NomadBLDCData *data) : NomadBLDCTransitionEvent(data)
    {
    }

    // Stop state machine and cleans up
    bool Triggered()
    {
        // Check Watchdog (from PC) Timeout
        WatchdogRegister_t watchdog = data_->controller->GetWatchdog();
        if(watchdog.timeout > 0) // Watchdog Enabled.  Check it
        {
            uint32_t now = HAL_GetTick();
            uint32_t deadline = watchdog.command_time + watchdog.timeout;
            //Logger::Instance().Print("Check Timeout: %d : %d : %d\r\n", now, watchdog.command_time, deadline);
            if(now >= deadline) // Expired
            {
                 // Set Error Code Here
                 Logger::Instance().Print("TIMED OUT WATCHDOG!\r\n");
                 return true;
            }
        }

        
        // Check Control Deadline Event

        // Check E-Stop Event?

        // Check Gate Driver Fault

        // Check Over Temp Fault? Or Limit Control

        // Check Position Limit Fault?
        //std::cout << "Check: " << data_->control_mode << std::endl;
        // if (data_->controller->GetControlMode() == req_mode_)
        // {
        //     //std::cout << "Event ID: " << name_ << " is SET!" << std::endl;
        //     return true;
        // }
        return false;
    };

protected:

};
#endif // NOMADBLDC_FSM_NOMADBLDCFSM_H_
