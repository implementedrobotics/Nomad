/*
 * CANDevice.h
 *
 *  Created on: November 3, 2021
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

#ifndef CAN_CANDEVICE_H_
#define CAN_CANDEVICE_H_

// C System Files
#include <stdint.h>

// C++ System Files
#include <string>

// Project Includes

// CAN Driver Includes
#include <linux/can.h>

class CANDevice
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

    // struct Config_t
    // {
    //     uint32_t id;         // CAN ID 11-bit max is 0x7ff
    //     uint32_t bitrate;    // Nominal Bitrate
    //     uint32_t d_bitrate;   // Data Bitrate
    //     uint32_t mode_fd;     // FD mode or classic
    //     float sample_point;  // Nominal Bitrate Sample Point Target
    //     float d_sample_point;  // Data Sample Point Target
    // };

    struct CAN_msg_t
    {
        uint32_t id = 0x7FF; // 11-bit max is 0x7FF.  Default to lowest prio message id
        uint16_t length = kBufferSizeRX; // Max FD CAN Length
        uint8_t data[kBufferSizeRX];    // Buffer for message data
    };

    // TODO: Support Extended IDs
    // Constructors ( Default 250kbps without Bit rate switching)
    //FDCANDevice(FDCAN_GlobalTypeDef *FDCAN, uint32_t node_id = 0x123, uint32_t bitrate = 250000, uint32_t dbitrate = 250000);
    //FDCANDevice(FDCAN_GlobalTypeDef *FDCAN, Config_t config);
    CANDevice();
    
    // Open CAN Port
    // TODO: Make consistent with PCAN Proprietary Drivers
    // TODO: Bitrate here vs OS defined
    bool Open(const std::string &canid, int32_t timeout = 2);
    bool Open(uint16_t index, int32_t timeout = 2);
    
    // Close Can Port
    bool Close();

    // Send CAN Data Function
    bool Send(uint32_t dest_id, uint8_t *data, uint16_t length);

    // Send CAN Message Function
    bool Send(CAN_msg_t &msg);

    // Receive CAN Data Function
    bool Receive(uint8_t *data, uint16_t &length);

    // Receive CAN Message Function
    bool Receive(CAN_msg_t &msg);

private:

    // Calc Bit Timing Helper Function
    bool CalculateTimings();

    // SocketCAN
    int socket_;

    // CAN Transmit/Receive Header
    //FDCAN_TxHeaderTypeDef tx_header_;
    //FDCAN_RxHeaderTypeDef rx_header_;

    // CAN Receive Message Container
    CAN_msg_t can_msg_;

    // Data Buffer
    //uint8_t can_rx_buffer_[kBufferSizeRX]; 

    // Config
    //Config_t config_;

    // Interrupts Enabled?
    bool enable_interrupt_;

    // Valid Timings
    bool timings_valid_;

    // Receive Timeout
    int32_t rx_timeout_;

};

#endif // CORE_PERIPHERAL_FDCAN_H_