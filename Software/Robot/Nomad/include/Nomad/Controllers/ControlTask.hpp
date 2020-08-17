/*
 * ControlTask.hpp
 *
 *  Created on: August 17, 2020
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

#ifndef ROBOT_NOMAD_CONTROLLERS_CONTROLTASK_H_
#define ROBOT_NOMAD_CONTROLLERS_CONTROLTASK_H_

// C System Files

// C++ System Files
#include <iostream>
#include <string>
#include <memory>

// Project Include Files
#include <Systems/BlockDiagram.hpp>

namespace Robot::Nomad::Controllers
{
    class ControlTask : public Core::Systems::BlockDiagram
    {

    public:
        // Block Diagram Class For Systems Task Node
        // name = Task Name
        // T_s = Sample Time (-1 for inherit)
        ControlTask(const std::string &name, const double T_s = -1);

        // Create Factory Function

    protected:

        // Overriden Run Function
        virtual void Run();

        // Pre-Run Setup Routine.  Setup any one time initialization here.
        virtual void Setup();

    };
} // namespace Core::Systems

#endif // ROBOT_NOMAD_CONTROLLERS_CONTROLTASK_H_
