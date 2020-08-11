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
#include <Common/State.hpp>
namespace Common
{
    State::State(const std::string &name, std::size_t id)
        : name_(name),
          id_(id),
          start_time_(0.0),
          end_time_(0.0),
          elapsed_time_(0.0),
          cycle_count_(0)
    {
        // Nothing to do
    }

    void State::Setup()
    {
        // Nothing to do
    }

    void State::Run(double dt)
    {

        elapsed_time_ += dt;
        cycle_count_++;

        Run_(dt);
    }

    void State::Enter(double current_time)
    {
        start_time_ = current_time;
        elapsed_time_ = 0.0;
        cycle_count_ = 0;

        Enter_(current_time);
    }
    // Called upon a state change and we enter this state
    // current_time = current robot/controller time
    void State::Enter_(double current_time)
    {
        // No op
    }

    void State::Exit(double current_time)
    {
        // Nothing to do
        end_time_ = current_time;
    }
    void State::AddTransitionEvent(TransitionEventPtr event, StatePtr next_state)
    {
        transition_map_.emplace(event, next_state);
    }
    bool State::ReadyToTransition()
    {
        // Loop transition maps for active events and then start transition
        for (auto transition : transition_map_)
        {
            TransitionEventPtr event = transition.first;
            if (event->Triggered())
            {
                next_state_ = transition.second;
                return true;
            }
        }
        return false;
    }

    // Next State to transition to
    const StatePtr &State::NextState() const
    {
        return next_state_;
    }

} // namespace Common