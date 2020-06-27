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

#ifndef NOMAD_COMMON_FINITESTATEMACHINE_H_
#define NOMAD_COMMON_FINITESTATEMACHINE_H_

// C System Files

// C++ System Files
#include <memory>
#include <string>
#include <vector>

// Third Party Includes

// Project Include Files
#include <Common/State.hpp>

namespace Common
{
  //class FiniteStateMachine;
  // Pointer type definition
  //using FiniteStateMachinePtr = std::shared_ptr<FiniteStateMachine>;

  // Finite State Machine Class
  class FiniteStateMachine
  {
  public:
    // Base Class Finite State Machine
    // name = Finite State Machine Name
    FiniteStateMachine(const std::string &name);

    // Get the name of this state machine
    inline const std::string& GetName() const
    {
      return name_;
    }

    // Add a new state to the state machine
    // state = Pointer to state object
    bool AddState(StatePtr state);

    // Sets initial state of the state machine
    // state = Pointer to state object
    bool SetInitialState(StatePtr state);

    // Reset state machine to initial start conditions
    virtual bool Reset(double current_time);

    // Initializes the state machine and runs any setup code needed before state machine
    // can be ran
    virtual bool Start(double current_time);

    // Run an iteration of the state machine
    virtual bool Run(double dt);

    // Stop state machine and cleans up
    virtual bool Stop(double current_time);

    // Transition to state
    // state_id = State id of requested state transition
    void TransitionTo(std::size_t state_id);

    // Transition to state
    // state_name = State name of requested state transition
    void TransitionTo(const std::string &state_name);

    // Transition to state
    // state_id = State id of requested state transition
    void TransitionTo(const StatePtr &state);

    // Create new state machine object
    // name = Finite State Machine name
    static std::shared_ptr<FiniteStateMachine> Create(const std::string& name);

  protected:
    // State machine name
    std::string name_;

    // Current state of the state machine
    StatePtr current_state_;

    // Initial state of the state machine when started
    StatePtr initial_state_;

    // Start time of state machine
    double start_time_;

    // End time of state machine
    double end_time_;

    // Total elapsed time since state machine began
    double elapsed_time_;

    // Total iteration count of state machine run cycles
    std::size_t cycle_count_;

    // List of states in this state machine
    std::vector<StatePtr> state_list_;

  private:
    // Find State
    // name = State name of state to find
    StatePtr FindState(const std::string &name);

    // Checks for state being a valid state
    // state = State to check for validity
    bool ValidState(const StatePtr &state);

  };

} // namespace Common

#endif // NOMAD_COMMON_FINITESTATEMACHINE_H_