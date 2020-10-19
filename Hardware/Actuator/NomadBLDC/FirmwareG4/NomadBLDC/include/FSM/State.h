/*
 * State.h
 *
 *  Created on: June 21, 2020
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

#ifndef NOMADBLDC_FSM_STATE_H_
#define NOMADBLDC_FSM_STATE_H_

// C System Files

// C++ System Files
#include <memory>
#include <string>
#include <map>

// Third Party Includes

// Project Include Files
#include <FSM/TransitionEvent.h>

// State Class
class State
{
public:
    // Base Class State
    // name = State name
    // id = State id
    State(const std::string &name, std::size_t id);

    // Get the name of this state
    inline const std::string &GetName() const
    {
        return name_;
    }

    // Get the id of this state
    inline const uint8_t &id() const
    {
        return id_;
    }

    // Check if state is ready/needs to transition
    virtual bool ReadyToTransition();

    // Add state transition event
    // event = Event to cause a state transition
    // next_state = next state to transition to upon event being active
    void AddTransitionEvent(TransitionEvent* event, State* next_state);

    // Initial first time state setup
    virtual void Setup();

    // Called upon a state change and we enter this state
    // current_time = current controller tick time
    virtual void Enter(uint32_t current_time);

    // current_time = current controller tick time
    // Called upon a state change and we are exiting this state
    virtual void Exit(uint32_t current_time);

    // Logic to run each iteration of the state machine run
    // dt = time step for this iteration
    virtual void Run(float dt);

    // Next State to transition to
    State* NextState() const;

    // TODO: Valid state transition?

protected:
    // Logic to run each iteration of the state machine run
    // dt = time step for this iteration
    virtual void Run_(float dt) = 0;

    // Called upon a state change and we enter this state
    // current_time = current controller tick time
    virtual void Enter_(uint32_t current_time);

    // Called upon a state change and we exit this state
    // current_time = current controller tick time
    virtual void Exit_(uint32_t current_time);

    // State name
    std::string name_;

    // State id
    uint8_t id_; // Can't see more than 256 states?

    // Next state
    State* next_state_;
    //bool in_transition_; // In Transition

    // Transition Event Map
    std::map<TransitionEvent*, State*> transition_map_;

    // Start begin time
    uint32_t start_time_;

    // End time of state machine
    uint32_t end_time_;

    // Total elapsed time since state machine began
    uint32_t elapsed_time_;

    // Total iteration count of state machine run cycles
    uint32_t cycle_count_;
};
#endif // NOMADBLDC_FSM_STATE_H_