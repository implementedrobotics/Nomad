/*
 * fdcan.h
 *
 *  Created on: November 3, 2020
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

#ifndef CORE_PERIPHERAL_FDCAN_H_
#define CORE_PERIPHERAL_FDCAN_H_

// C System Files

// C++ System Files

// STM32 System Files
#include <main.h>

// Project Includes

class FDCANDevice
{

public:

    // TODO: This should be up in a base "Peripheral Class"
    static constexpr int kMaxInterrupts = 200;

    // TODO: Interrupted vs Polled. 
    // Assumes a pre "inited" FDCAN from CubeMX

    // Constructor
    FDCANDevice(FDCAN_GlobalTypeDef *FDCAN);

    // Enable FDCAN
    void Enable();

    // Enable Interrupt
    void EnableIT();

    void Send(uint8_t *data, uint16_t length);

    void Receive();

    // TODO: This should be up in a base "Peripheral Class"
    inline void ISR() 
    {

    }

private:

    // STM32 FDCAN Type
    FDCAN_GlobalTypeDef *FDCAN_;
    FDCAN_HandleTypeDef hfdcan_;

    // Interrupts Enabled?
    bool enable_interrupt_;

    // ISR Number
    IRQn_Type IRQn_; 

    // ISR Table
    //static ADCDevice* ISR_VTABLE[kMaxInterrupts];
};

#endif // CORE_PERIPHERAL_FDCAN_H_