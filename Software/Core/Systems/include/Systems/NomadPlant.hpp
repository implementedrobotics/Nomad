/*
 * NomadPlant.hpp
 *
 *  Created on: August 5, 2019
 *      Author: Quincy Jones
 *
 * Copyright (c) <2019> <Quincy Jones - quincy@implementedrobotics.com/>
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

#ifndef NOMAD_CORE_SYSTEMS_NOMADPLANT_H_
#define NOMAD_CORE_SYSTEMS_NOMADPLANT_H_

// C System Files
#include <stdint.h>

// C++ System Files
#include <iostream>
#include <string>
#include <memory>

// Third Party Includes
#include <Realtime/Messages/double_vec_t.hpp>

// Project Includes
#include <Realtime/RealTimeTask.hpp>
#include <Systems/RigidBody.hpp>

namespace Systems
{
namespace Nomad
{
class NomadPlant : public Realtime::RealTimeTaskNode
{

public:
    enum OutputPort
    {
        STATE = 0 // State Output
    };

    enum InputPort
    {
        FORCES = 0, // Input Forces
    };

    // Base Class Reference Nomad Plant Task Node
    // name = Task Name
    NomadPlant(const std::string &name);

protected:
    // Overriden Run Function
    virtual void Run();

    // Pre-Run Setup Routine.  Setup any one time initialization here.
    virtual void Setup();

    // Input (State Estimate)
    double_vec_t forces_in;

    // TODO:
    // Dynamic System Block
    RigidBlock1D block_;


};
} // namespace Nomad
} // namespace Systems

#endif // NOMAD_CORE_SYSTEMS_NOMADPLANT_H_
