/*
 * FiniteStateMachine.h
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

#ifndef NOMADBLDC_FSM_FINITESTATEMACHINE_H_
#define NOMADBLDC_FSM_FINITESTATEMACHINE_H_

// C System Files

// C++ System Files
#include <memory>
#include <string>
#include <vector>

// Third Party Includes

// Project Include Files
#include <FSM/State.h>

// Finite State Machine Class
class FiniteStateMachine
{
public:
    // Base Class Finite State Machine
    FiniteStateMachine();

    // Add a new state to the state machine
    // state = Pointer to state object
    bool AddState(State *state);

    // Sets initial state of the state machine
    // state = Pointer to state object
    bool SetInitialState(State *state);

    // Reset state machine to initial start conditions
    virtual bool Reset(uint32_t current_time);

    // Run Initilization Setup, Allocate and Memory Etc
    virtual bool Setup();

    // Initializes the state machine and runs any setup code needed before state machine
    // can be ran
    virtual bool Start(uint32_t current_time);

    // Run an iteration of the state machine
    virtual bool Run(float dt);

    // Stop state machine and cleans up
    virtual bool Stop(uint32_t current_time);

    // Transition to state
    // state_id = State id of requested state transition
    void TransitionTo(State *state);

    // Create new state machine object
    // name = Finite State Machine name
    static FiniteStateMachine *Create();

protected:

    // Current state of the state machine
    State *current_state_;

    // Initial state of the state machine when started
    State *initial_state_;

    // Start time of state machine
    uint32_t start_time_;

    // End time of state machine
    uint32_t end_time_;

    // Total elapsed time since state machine began
    uint32_t elapsed_time_;

    // Total iteration count of state machine run cycles
    uint32_t cycle_count_;

    // List of states in this state machine
    std::vector<State *> state_list_;

private:
    // Find State
    // name = State name of state to find
    //State *FindState(const std::string &name);

    // Checks for state being a valid state
    // state = State to check for validity
    bool ValidState(const State *state);
};

#endif // NOMADBLDC_FSM_FINITESTATEMACHINE_H_