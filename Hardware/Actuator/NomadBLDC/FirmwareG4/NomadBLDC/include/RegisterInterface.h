/*
 * RegisterInterface.h
 *
 *  Created on: November 7, 2020
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

#ifndef REGISTER_INTERFACE_H_
#define REGISTER_INTERFACE_H_

// C System Files

// C++ System Files

// Project Includes
#include <Peripherals/uart.h>
#include <Peripherals/fdcan.h>

class RegisterInterface
{
public:
    static constexpr uint16_t kMaxRegisters = (1<<12);

    // TODO: Address field is too large.  But keeping clean alignment math here
    struct register_command_t
    {
        uint32_t rwx: 2;      // Read/Write/Execute
        uint32_t address: 12; // 12-bit address (4096 Max Addresses)    
        uint32_t data_type: 2; // Data Type: 12-bit fixed, 16-bit fixed, 32-bit fixed, 32-bit float
        uint8_t  data[62];     // 62 byte data field
    };

    typedef enum
    {
        MotorStatus = 0x00,
        ControllerStatus = 0x01,
    } RegisterAddress_e;


    static void HandleCommand(FDCANDevice::FDCAN_msg_t &command);

    RegisterInterface();

    // Register Offset Map
    // Store Base Memory Address
    // Register Will Offset From There
    // Support Float Return Precision Options

private:

    uint32_t register_map_[kMaxRegisters];

};     // namespace RegisterInterface
#endif // REGISTER_INTERFACE_H_