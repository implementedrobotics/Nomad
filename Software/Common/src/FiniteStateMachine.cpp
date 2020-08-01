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
#include <iostream>

// Third Party Includes

// Project Include Files
#include <Common/FiniteStateMachine.hpp>

namespace Common
{

    FiniteStateMachine::FiniteStateMachine(const std::string &name) 
    : name_(name), 
    current_state_(nullptr),
    initial_state_(nullptr),
    start_time_(0.0),
    end_time_(0.0),
    elapsed_time_(0.0),
    cycle_count_(0.0)
    {
        // Nothing to do here
    }

    bool FiniteStateMachine::AddState(StatePtr state)
    {
        state_list_.push_back(state);
        return true;
    }

    bool FiniteStateMachine::SetInitialState(StatePtr state)
    {
        initial_state_ = current_state_ = state;
    }

    bool FiniteStateMachine::Reset(double current_time)
    {
        if (current_state_ != nullptr)
            current_state_->Exit(current_time); // Cleanup old state

        return Start(current_time);
    }

    bool FiniteStateMachine::Start(double current_time)
    {
        cycle_count_ = 0;

        // TODO: Should be get time from System
        start_time_ = current_time;
        elapsed_time_ = 0.0;

        if(current_state_ != nullptr)
        {
            current_state_->Exit(current_time);
        }
        current_state_ = initial_state_;
        current_state_->Enter(current_time);
    }

    bool FiniteStateMachine::Stop(double current_time)
    {
        // 
        end_time_ = current_time;//start_time_ + elapsed_time_;
    }
    bool FiniteStateMachine::Run(double dt)
    {
         if (current_state_ == nullptr)
         {
             std::cout << "[ERROR]: Current state is NULL";
             return false;
         }

        if(current_state_->ReadyToTransition())
        {
            //std::cout << "State ready to transition" << std::endl;
            TransitionTo(current_state_->NextState());
        }
                // else if (current_state_->InTransition()) // In Transition, run code
        // {
        //     std::cout << "In state transition next state" << std::endl;
        //     // TODO: Call state transition callbackk
        //     //current_state_->RunTransition()
        // }
        else
        {
            current_state_->Run(dt);
        }
        
        elapsed_time_ += dt;
        cycle_count_++;
    }
    void FiniteStateMachine::TransitionTo(const StatePtr &state)
    {
        std::cout << "Transition From: [" << current_state_->GetName() << "] --> [" << state->GetName() << "]" << std::endl;
        current_state_->Exit(elapsed_time_);

        current_state_ = state;

        current_state_->Enter(elapsed_time_);

    }
    void FiniteStateMachine::TransitionTo(std::size_t state)
    {
        //current_state_->Transition(FindState(state));
    }

    StatePtr FiniteStateMachine::FindState(const std::string &name)
    {
        // TODO: Check for valid key
        return nullptr;//state_list_[eStateId];
    }

    std::shared_ptr<FiniteStateMachine> FiniteStateMachine::Create(const std::string& name)
    {
      return std::make_shared<FiniteStateMachine>(name);
    };
} // namespace Common
