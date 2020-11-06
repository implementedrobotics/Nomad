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

    // Const Expressions
    static constexpr uint16_t kMaxNominalPrescaler = 512;
    static constexpr uint16_t kMaxNominalTimeSeg1 = 256;
    static constexpr uint16_t kMaxNominalTimeSeg2 = 128;
    static constexpr uint16_t kMinNominalTimeSeg = 2;

    static constexpr uint16_t kMaxDataPrescaler = 32;
    static constexpr uint16_t kMaxDataTimeSeg1 = 32;
    static constexpr uint16_t kMaxDataTimeSeg2 = 16;
    static constexpr uint16_t kMinDataTimeSeg = 1;

    // TODO: This should be up in a base "Peripheral Class"
    static constexpr int kMaxInterrupts = 200;

    struct Config_t
    {
        uint32_t id;       // CAN ID
        uint32_t bitrate;  // Nominal Bitrate
        uint32_t dbitrate; // Data Bitrate
        uint8_t mode_fd;   // FD mode or classic
        float sp;          // Nominal Bitrate Sample Point Target
        float data_sp;     // Data Sample Point Target
    };

    // TODO: Interrupted vs Polled. 
    // Assumes a pre "inited" FDCAN from CubeMX

    // Constructor
    FDCANDevice(FDCAN_GlobalTypeDef *FDCAN);

    // Init CAN
    bool Init();

    // Enable FDCAN
    bool Enable();

    // Disable FDCAN
    void Disable();

    // Enable Interrupt
    void EnableIT();

    void Send(uint32_t dest_id, uint8_t *data, uint16_t length);

    void Receive();

    // TODO: This should be up in a base "Peripheral Class"
    inline void ISR() 
    {

    }

private:

    // Calc Bit Timing Helper Function
    bool CalculateTimings();

    // STM32 FDCAN Type
    FDCAN_GlobalTypeDef *FDCAN_;
    FDCAN_HandleTypeDef hfdcan_;

    // Transmit Header
    FDCAN_TxHeaderTypeDef tx_header_;
    FDCAN_RxHeaderTypeDef rx_header_;

    // Config
    Config_t config_;

    // Interrupts Enabled?
    bool enable_interrupt_;

    // Valid Timings
    bool timings_valid_;

    // ISR Number
    IRQn_Type IRQn_; 

    // ISR Table
    //static ADCDevice* ISR_VTABLE[kMaxInterrupts];

   
};

#endif // CORE_PERIPHERAL_FDCAN_H_