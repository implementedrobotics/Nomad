/*
 * NomadState.hpp
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

#ifndef NOMAD_FSM_NOMADSTATE_H_
#define NOMAD_FSM_NOMADSTATE_H_

// C System Files

// C++ System Files

// Third Party Includes

// Project Include Files
#include <Common/State.hpp>
#include <Nomad/FSM/NomadControlData.hpp>
namespace Robot::Nomad::FSM
{
    // Base NomadState Class
    class NomadState : public Common::State
    {
    public:
        // Base Class NomadState
        NomadState(const std::string &name, std::size_t id) : Common::State(name, id)
        {
        }

        void SetControllerData(std::shared_ptr<NomadControlData> data)
        {
            data_ = data;
        }
        
    protected:
        // Data pointer to controller data pointer
        std::shared_ptr<NomadControlData> data_;
    };

} // namespace Robot::Nomad::FSM
#endif // NOMAD_FSM_NOMADSTATE_H_