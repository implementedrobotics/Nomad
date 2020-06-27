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
#include <linux/joystick.h>

// C++ System Files
#include <iostream>
#include <string>

// Project Includes

#define NUM_AXES 8
#define NUM_BUTTONS 11

namespace OperatorInterface
{
    namespace Teleop
    {
        class GamepadInterface
        {
        public:
            struct AxisState
            {
                float value;
                //float y;
            };

            struct ButtonState
            {
                bool pressed;
                bool released;
            };

            struct DPadState
            {
                bool left;
                bool right;
                bool up;
                bool down;
            };

            // Button Mappings
            enum ButtonType
            {
                BUTTON_A = 0,
                BUTTON_B = 1,
                BUTTON_X = 2,
                BUTTON_Y = 3,
                BUTTON_LB = 4,
                BUTTON_RB = 5,
                BUTTON_BACK = 6,
                BUTTON_START = 7,
                BUTTON_HOME = 8,
                BUTTON_LEFT_STICK = 9,
                BUTTON_RIGHT_STICK = 10
            };

            // Analog Mappings
            enum AxisType
            {
                ANALOG_LEFT_STICK_X = 0,
                ANALOG_LEFT_STICK_Y = 1,
                ANALOG_LEFT_TRIGGER = 2,
                ANALOG_RIGHT_STICK_X = 3,
                ANALOG_RIGHT_STICK_Y = 4,
                ANALOG_RIGHT_TRIGGER = 5,
            };

            // D-Pad Mappings
            enum DPadType
            {
                D_PAD_LEFT_RIGHT = 6,
                D_PAD_UP_DOWN = 7,
                D_PAD_LEFT = 8,
                D_PAD_RIGHT = 9,
                D_PAD_UP = 10,
                D_PAD_DOWN = 11
            };

            // Base Class Gamepad Interface
            // device = Device driver name, e.g. /dev/input/js0
            GamepadInterface(const std::string &device);
            GamepadInterface();
            ~GamepadInterface();

            // Poll Gamepad Interface for new events
            void Poll();

            // Check button state
            bool IsPressed(ButtonType button);

            // Check button state
            bool IsReleased(ButtonType button);

            float GetValue(AxisType axis);

            // Get Button State
            ButtonState GetButtonState(ButtonType button);

            // Return Axis State
            AxisState GetAxisState(AxisType axis);

            // Return State of DPAD
            bool GetDPadState(DPadType dpad);

            // Open Device
            void OpenDevice(const std::string &device);

        private:
            // Reset gamepad state
            void Reset();

            // Axis State
            AxisState axis_[NUM_AXES];

            // Button State
            ButtonState buttons_[NUM_BUTTONS];

            // Directional Pad Status
            DPadState dpad_;

            // Device
            std::string device_;

            // File descriptor holder
            int fd_;

            // Is driver open?
            bool is_open_;

            // Joystick event struct
            struct js_event js_event_;

            // Minimum Axis Value
            static const short MIN_AXES_VALUE = -32768;
            
            // Maximum AXis Value
            static const short MAX_AXES_VALUE = 32767;
        };
    } // namespace Teleop
} // namespace OperatorInterface

#endif // NOMAD_OPERATORINTERFACE_GAMEPAD_H_
