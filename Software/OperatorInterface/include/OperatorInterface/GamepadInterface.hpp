/*
 * GamepadInterface.hpp
 *
 *  Created on: June 26, 2020
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

#ifndef NOMAD_OPERATORINTERFACE_GAMEPAD_H_
#define NOMAD_OPERATORINTERFACE_GAMEPAD_H_

// C System Files

// C++ System Files
#include <iostream>
#include <string>

// Project Includes

namespace OperatorInterface
{
    namespace Teleop
    {
        class GamepadInterface
        {

        public:
            // Base Class Gamepad Interface
            GamepadInterface();
            ~GamepadInterface();

        private:
            bool OpenDevice(const std::string& device);
        
            // File descriptor holder
            int fd_;

        };
    } // namespace Teleop
} // namespace OperatorInterface

#endif // NOMAD_OPERATORINTERFACE_GAMEPAD_H_
