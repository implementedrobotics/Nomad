/*
 * RemoteTeleop.cpp
 *
 *  Created on: July 17, 2019
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

// Primary Include
#include <Nomad/OperatorInterface/GamepadInterface.hpp>

// C System Includes
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

// C++ System Includes
#include <iostream>
#include <string>
#include <cstring>

// Third-Party Includes

// Project Includes

namespace OperatorInterface::Teleop
{
    GamepadInterface::GamepadInterface() : is_open_(false), fd_(-1)
    {
        Reset();
    }
    GamepadInterface::GamepadInterface(const std::string &device) : is_open_(false), fd_(-1), device_(device)
    {
        OpenDevice(device_);
    }

    GamepadInterface::~GamepadInterface()
    {
        if (is_open_)
        {
            close(fd_);
        }
    }

    void GamepadInterface::OpenDevice(const std::string &device)
    {
        device_ = device;
        Reset();
        // Close any already open devices
        if (is_open_)
        {
            close(fd_);
        }
        std::cout << "[GamepadInterface]: Opening Gamepad Device: " << device_ << std::endl;

        // Open Joystick w/ Non blocking
        fd_ = open(device_.c_str(), O_RDONLY | O_NONBLOCK);

        if (fd_ != -1)
            is_open_ = true;

        if (!is_open_)
        {
            std::cout << "[GamepadInterface]: ERROR opening Gamepad Device: " << device_ << std::endl;
            return;
        }

        std::cout << "[GamepadInterface]: Successfully opened Gamepad Device: " << device_ << std::endl;

        Poll();
    }

    void GamepadInterface::Poll()
    {
        while (read(fd_, &js_event_, sizeof(struct js_event)) == sizeof(struct js_event))
        {
            switch (js_event_.type)
            {

            case JS_EVENT_BUTTON:
                buttons_[js_event_.number].pressed = js_event_.value ? true : false;
                buttons_[js_event_.number].released = js_event_.value ? false : true;
                break;

            case JS_EVENT_AXIS:
                // Map Trigger from 0 to 1
                if (js_event_.number == AxisType::ANALOG_LEFT_TRIGGER || js_event_.number == AxisType::ANALOG_RIGHT_TRIGGER)
                {
                    axis_[js_event_.number].value = (float)(js_event_.value + MAX_AXES_VALUE) / (2 * MAX_AXES_VALUE);
                }
                else if (js_event_.number == DPadType::D_PAD_LEFT_RIGHT) // Map DPAD Axis
                {
                    if (js_event_.value == 0)
                    {
                        dpad_.left = false;
                        dpad_.right = false;
                    }
                    else if (js_event_.value < 0)
                    {
                        dpad_.left = true;
                        dpad_.right = false;
                    }
                    else
                    {
                        dpad_.left = false;
                        dpad_.right = true;
                    }
                }
                else if (js_event_.number == DPadType::D_PAD_UP_DOWN) // Map DPAD Axis
                {
                    if (js_event_.value == 0)
                    {
                        dpad_.up = false;
                        dpad_.down = false;
                    }
                    else if (js_event_.value < 0)
                    {
                        dpad_.up = true;
                        dpad_.down = false;
                    }
                    else
                    {
                        dpad_.up = false;
                        dpad_.down = true;
                    }
                }
                else // Map everything else from -1.0 to 1.0
                    axis_[js_event_.number].value = (float)js_event_.value / MAX_AXES_VALUE;
                break;

            default:
                // Do Nothing for Events
                break;
            }
        }
    }

    GamepadInterface::ButtonState GamepadInterface::GetButtonState(ButtonType button)
    {
        return buttons_[button];
    }

    GamepadInterface::AxisState GamepadInterface::GetAxisState(AxisType axis)
    {
        return axis_[axis];
    }

    bool GamepadInterface::GetDPadState(DPadType dpad)
    {

        switch (dpad)
        {
        case DPadType::D_PAD_LEFT:
            return dpad_.left;
        case DPadType::D_PAD_RIGHT:
            return dpad_.right;
        case DPadType::D_PAD_UP:
            return dpad_.up;
        case DPadType::D_PAD_DOWN:
            return dpad_.down;
        default:
            return false;
        }
    }

    bool GamepadInterface::IsPressed(ButtonType button)
    {
        return buttons_[button].pressed;
    }

    bool GamepadInterface::IsReleased(ButtonType button)
    {
        return buttons_[button].released;
    }

    float GamepadInterface::GetValue(AxisType axis)
    {
        return axis_[axis].value;
    }

    void GamepadInterface::Reset()
    {
        std::memset(&axis_, 0, sizeof(AxisState) * NUM_AXES);
        std::memset(&buttons_, 0, sizeof(ButtonState) * NUM_BUTTONS);
        std::memset(&dpad_, 0, sizeof(DPadState));
    }
} // namespace OperatorInterface::Teleop
