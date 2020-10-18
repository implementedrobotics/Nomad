  
/*
 * FiniteStateMachine.cpp
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

// C System Files

// C++ System Files

// Third Party Includes

// Project Include Files
#include <main.h>
#include <FSM/FiniteStateMachine.h>

FiniteStateMachine::FiniteStateMachine(const std::string &name)
    : name_(name),
      current_state_(nullptr),
      initial_state_(nullptr),
      start_time_(0),
      end_time_(0),
      elapsed_time_(0),
      cycle_count_(0)
{
    // Nothing to do here
}

bool FiniteStateMachine::AddState(State* state)
{
    state_list_.push_back(state);
    return true;
}

bool FiniteStateMachine::SetInitialState(State* state)
{
    initial_state_ = current_state_ = state;
    return true;
}

bool FiniteStateMachine::Reset(uint32_t current_time)
{
    if (current_state_ != nullptr)
        current_state_->Exit(current_time); // Cleanup old state

    return Start(current_time);
}

bool FiniteStateMachine::Start(uint32_t current_time)
{
    cycle_count_ = 0;

    // TODO: Should be get time from System
    start_time_ = current_time;
    elapsed_time_ = 0;

    if (current_state_ != nullptr)
    {
        current_state_->Exit(current_time);
    }
    current_state_ = initial_state_;
    current_state_->Enter(current_time);

    return true;
}

bool FiniteStateMachine::Stop(uint32_t current_time)
{
    end_time_ = current_time;
    return true;
}

bool FiniteStateMachine::Run(double dt)
{
    if (current_state_ == nullptr)
    {
        return false;
    }

    if (current_state_->ReadyToTransition())
    {
        TransitionTo(current_state_->NextState());
    }
    else
    {
        current_state_->Run(dt);
    }

    elapsed_time_ += SysTick->VAL - start_time_;
    cycle_count_++;
    return true;
}
void FiniteStateMachine::TransitionTo(State* state)
{
    // Exit Current State
    current_state_->Exit(elapsed_time_);

    // Update Current State to parameter
    current_state_ = state;

    // Enter Next State
    current_state_->Enter(elapsed_time_);
}

FiniteStateMachine* FiniteStateMachine::Create(const std::string &name)
{
    return new FiniteStateMachine(name);
};