/*
 * TransitionEvent.h
 *
 *  Created on: June 22, 2020
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

#ifndef NOMADBLDC_FSM_TRANSITIONEVENT_H_
#define NOMADBLDC_FSM_TRANSITIONEVENT_H_

// C System Files

// C++ System Files
#include <memory>
#include <string>
#include <vector>

// Third Party Includes

// Project Include Files
#include <FSM/State.h>

// Transition Event Class
class TransitionEvent
{
public:
    // Base Class Transition Event
    // name = Transition Event name
    TransitionEvent(/* const std::string &name */);

    // // Get the name of this transition event
    // inline const std::string &GetName() const
    // {
    //     return name_;
    // }

    // Stop state machine and cleans up
    virtual bool Triggered() = 0;

protected:
    // // State machine name
    // std::string name_;
};

#endif // NOMADBLDC_FSM_TRANSITIONEVENT_H_
