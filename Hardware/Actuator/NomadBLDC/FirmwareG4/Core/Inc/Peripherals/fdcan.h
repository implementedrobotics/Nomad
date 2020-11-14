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
#include <functional>

// STM32 System Files
#include <main.h>

// Project Includes
#include <nomad_hw.h>

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

    // Receive Buffer Size. For now match max CANFD Frame Size
    // TODO: Support CAN Classic + Multi Packet 
    static constexpr uint16_t kBufferSizeRX = 64;

    // TODO: This should be up in a base "Peripheral Class"
    static constexpr int kMaxInterrupts = 127;

    // Length to Data Length Code Conversion LUT
    static constexpr uint8_t DLC_LUT[] = {
        // 1 to 8 Bytes
        0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8,
        // 9 to 12 Bytes
        0x9, 0x9, 0x9, 0x9,
        // 13 to 16 Bytes
        0xA, 0xA, 0xA, 0xA,
        // 17 to 20 Bytes
        0xB, 0xB, 0xB, 0xB,
        // 21 to 24 Bytes
        0xC, 0xC, 0xC, 0xC,
        // 25 to 32 Bytes
        0xD, 0xD, 0xD, 0xD, 0xD, 0xD, 0xD, 0xD,
        // 33 to 48 Bytes
        0xE, 0xE, 0xE, 0xE, 0xE, 0xE, 0xE, 0xE,
        0xE, 0xE, 0xE, 0xE, 0xE, 0xE, 0xE, 0xE,
        // 49 to 64 Bytes
        0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF,
        0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF};

    // Data Length Code to Length Conversion LUT
    static constexpr uint8_t LEN_LUT[] = {
        0, 1, 2, 3, 4, 5, 6, 7,
        8, 12, 16, 20, 24, 32, 48, 64};

    struct Config_t
    {
        uint32_t id;         // CAN ID 11-bit max is 0x7ff
        uint32_t bitrate;    // Nominal Bitrate
        uint32_t d_bitrate;   // Data Bitrate
        uint8_t mode_fd;     // FD mode or classic
        float sample_point;  // Nominal Bitrate Sample Point Target
        float d_sample_point;  // Data Sample Point Target
    };

    struct FDCAN_msg_t
    {
        uint32_t id = 0x7FF; // 11-bit max is 0x7FF.  Default to lowest prio message id
        uint16_t length = kBufferSizeRX; // Max FD CAN Length
        uint8_t data[kBufferSizeRX];    // Buffer for message data
    };

    // TODO: Support Extended IDs
    // Constructors ( Default 250kbps without Bit rate switching)
    FDCANDevice(FDCAN_GlobalTypeDef *FDCAN, uint32_t node_id = 0x123, uint32_t bitrate = 250000, uint32_t dbitrate = 250000);
    FDCANDevice(FDCAN_GlobalTypeDef *FDCAN, Config_t config);

    // Init CAN
    bool Init();

    // Enable FDCAN
    bool Enable();

    // Disable FDCAN
    void Disable();

    // Enable Interrupt
    void EnableIT();

    // Set Complete Callback
    // TODO: We have 2 interrupt lines.  For now attach only to 0
    void Attach(const std::function<void(FDCAN_msg_t&)> &recv_cb)
    {
        recv_callback_ = recv_cb;
    }

    // Send CAN Message Function
    bool Send(uint32_t dest_id, uint8_t *data, uint16_t length) CCM_ATTRIBUTE;

    // Receive CAN Data Function
    bool Receive(uint8_t *data, uint16_t &length) CCM_ATTRIBUTE;

    // Receive CAN Message Function
    bool Receive(FDCAN_msg_t &msg) CCM_ATTRIBUTE;

    // TODO: This should be up in a base "Peripheral Class"
    // TODO: Also for FDCAN this is ONLY a receive interrupt supporting function.
    // TODO: Eventually make this handle all things interrupt...
    inline void ISR() 
    {
        // Read Message
        if ((GetRxFIFOITFlags() & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
        {
            // Get Message From Buffer
            Receive(fdcan_msg_.data, fdcan_msg_.length);

            // Execute Callback ( Forward Message to Handler )
            recv_callback_(fdcan_msg_);
        }

        // Handle clearing registers etc
        // TODO: As usual HAL IRQ handlers are way overbloated.  
        // TODO: Optimize register clearing etc.
        HAL_FDCAN_IRQHandler(hfdcan_);
    }
    inline uint32_t GetRxFIFOITFlags()
    {
        uint32_t RxFifo0ITs = FDCAN_->IR & (FDCAN_IR_RF0L | FDCAN_IR_RF0F | FDCAN_IR_RF0N);
        RxFifo0ITs &= FDCAN_->IE;
        return RxFifo0ITs;
    }

    // Return Handle to FDCAN TypeDef
    inline FDCAN_HandleTypeDef* Handle() { return hfdcan_; };

private:

    // Calc Bit Timing Helper Function
    bool CalculateTimings();

    // STM32 FDCAN Types
    FDCAN_GlobalTypeDef *FDCAN_;
    FDCAN_HandleTypeDef *hfdcan_;

    // CAN Transmit/Receive Header
    FDCAN_TxHeaderTypeDef tx_header_;
    FDCAN_RxHeaderTypeDef rx_header_;

    // CAN Receive Message Container
    FDCAN_msg_t fdcan_msg_;

    // Data Buffer
    //uint8_t can_rx_buffer_[kBufferSizeRX]; 

    // Config
    Config_t config_;

    // Interrupts Enabled?
    bool enable_interrupt_;

    // Valid Timings
    bool timings_valid_;

    // ISR Number
    IRQn_Type IRQn_; 

    // ISR Table
    //static FDCANDevice* ISR_VTABLE[kMaxInterrupts];

    // Interrupt Callback
    std::function<void(FDCAN_msg_t&)> recv_callback_ = [=](FDCAN_msg_t&) {};
};

#endif // CORE_PERIPHERAL_FDCAN_H_