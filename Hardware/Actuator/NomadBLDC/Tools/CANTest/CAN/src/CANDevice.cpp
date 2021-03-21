  
/*
 * CANDevice.cpp
 *
 *  Created on: March 7, 2021
 *      Author: Quincy Jones
 *
 * Copyright (c) <2021> <Quincy Jones - quincy@implementedrobotics.com/>
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


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <iostream>

#include <CAN/CANDevice.h>



void CANDevice::ReceiveTask()
{
    CAN_msg_t msg;
    while (1)
    {
        if (Receive(msg))
        {
           // std::cout << "RECEIVING CAN_DEV: " << msg.length << std::endl;
            for(auto listener : rx_listeners_) // Pass to any registered listeners
            {
                listener(msg);
            }
        }
    }
}

CANDevice::CANDevice()
{
}
CANDevice::~CANDevice()
{
    rx_thread_.detach();
}

bool CANDevice::StartReceiveThread()
{
    std::cout << "Starting Receive Thread" << std::endl;
    rx_thread_ = std::thread(&CANDevice::ReceiveTask, this);
    return true;
}
void CANDevice::RegisterListenerCB(const std::function<void(CAN_msg_t&)> &recv_cb)
{
    std::cout << "Registering Callback" << std::endl;
    rx_listeners_.push_back(recv_cb);
}
