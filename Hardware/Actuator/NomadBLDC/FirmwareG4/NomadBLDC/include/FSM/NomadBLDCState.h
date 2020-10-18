/*
 * NomadBLDCState.h
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

#ifndef NOMADBLDC_FSM_NOMADBLDCSTATE_H_
#define NOMADBLDC_FSM_NOMADBLDCSTATE_H_

// C System Files

// C++ System Files

// Third Party Includes

// Project Include Files
#include <FSM/State.h>
//#include <FSM/NomadControlData.h>
#include <FSM/NomadBLDCFSM.h>

// Base NomadState Class
class NomadBLDCState : public State
{
public:
    // Base Class NomadState
    NomadBLDCState(const std::string &name, std::size_t id) : State(name, id), parent_(nullptr)
    {
    }

    void SetParentFSM(NomadBLDCFSM* parent)
    {
        parent_ = parent;
    }

    // void SetControllerData(std::shared_ptr<NomadControlData> data)
    // {
    //     data_ = data;
    // }

protected:

    // Data pointer to controller data pointer
    //std::shared_ptr<NomadControlData> data_;

    // Parent State Machine Container for this state
    NomadBLDCFSM *parent_;
};
#endif // NOMAD_FSM_NOMADSTATE_H_