/*
 * RegisterInterface.cpp
 *
 *  Created on: March 20, 2020
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
 * 
 */

// Primary Include
#include "RegisterInterface.h"

// C System Files
#include <cstring>

// C++ System Files

// Project Includes
#include <Peripherals/uart.h>
#include <Peripherals/fdcan.h>   
#include <Logger.h>

//UARTDevice* RegisterInterface::gUART = 0;
RegisterInterface::RegisterInterface()
{
    
}

// void RegisterInterface::SetUART(UARTDevice *UART)
// {
//     gUART = UART;
// }
void RegisterInterface::HandleCommand(FDCANDevice::FDCAN_msg_t &command)
{
    // register_command_t reg;
    // reg.rwx = 3;
    // reg.address = 123;
    // reg.data_type = 2;

    // reg = (register_command_t)command.data;
    // Logger::Instance().Print("Handling: 0%x \r\n", reg);
}
