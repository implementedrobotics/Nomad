/*
 * PCANDevice.h
 *
 *  Created on: March 19, 2021
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
 * 
 */

#ifndef PCAN_CANDEVICE_H_
#define PCAN_CANDEVICE_H_

// C System Files
#include <stdint.h>

// C++ System Files
#include <string>

// Project Includes
#include <CAN/CANDevice.h>

class PCANDevice : public CANDevice
{

public:

    // TODO: Support Extended IDs
    PCANDevice();
    
    // Open CAN Port
    virtual bool Open(const std::string &device_id, Config_t &config, bool bUseRXThread = false);
    
    // Close Can Port
    virtual bool Close();

    // Filtering Add
    virtual bool AddFilter(uint32_t from, uint32_t to);

    // Filtering Clear
    virtual bool ClearFilters();

    // Send CAN Data Function
    virtual bool Send(uint32_t dest_id, uint8_t *data, uint16_t length);

    // Send CAN Message Function
    virtual bool Send(CAN_msg_t &msg);

    // Receive CAN Message Function
    virtual bool Receive(CAN_msg_t &msg);

protected:

    int fd_; // PCAN File Descriptor

};

#endif // PCAN_CANDEVICE_H_