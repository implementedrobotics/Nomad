/*
 * CANDevice.h
 *
 *  Created on: March 3, 2021
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
#include <thread>
#include <vector>
#include <functional>

// Project Includes

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

    struct Config_t
    {
        uint32_t id;         // CAN ID 11-bit max is 0x7ff
        uint32_t bitrate;    // Nominal Bitrate
        uint32_t d_bitrate;   // Data Bitrate
        uint32_t mode_fd;     // FD mode or classic
        float sample_point;  // Nominal Bitrate Sample Point Target
        float d_sample_point;  // Data Sample Point Target
        uint32_t clock_freq;   // CAN Device Clock HZ

        // TODO: Bittimings?
        uint16_t brp;   // Baud Rate Prescaler
        uint16_t tq;    // Time Quanta (ns)
        uint16_t tseg1; // Phase Time Segment 1
        uint16_t tseg2; // Phase Time Segment 2
        uint16_t sjw;   // Sync Jump Width

        uint16_t d_brp;   // Baud Rate Prescaler
        uint16_t d_tseg1; // Phase Time Segment 1
        uint16_t d_tseg2; // Phase Time Segment 2
        uint16_t d_sjw;   // Sync Jump Width
    };

    struct CAN_msg_t
    {
        uint32_t id = 0x7FF; // 11-bit max is 0x7FF.  Default to lowest prio message id
        uint16_t length = kBufferSizeRX; // Max FD CAN Length
        std::array<uint8_t, kBufferSizeRX> data;  // Buffer for message data
    };

    // TODO: Support Extended IDs
    //CANDevice();
    ~CANDevice();
    
    // Open CAN Port
    virtual bool Open(const std::string &device_id, Config_t &config, bool bUseRXThread = false) = 0;
    
    // Close Can Port
    virtual bool Close() = 0;

    // Filtering Add
    virtual bool AddFilter(uint32_t from, uint32_t to) = 0;

    // Filtering Clear
    virtual bool ClearFilters() = 0;

    // Send CAN Data Function
    virtual bool Send(uint32_t dest_id, uint8_t *data, uint16_t length) = 0;

    // Send CAN Message Function
    virtual bool Send(CAN_msg_t &msg) = 0;

    // Receive CAN Message Function
    virtual bool Receive(CAN_msg_t &msg) = 0;

    // Debug Bus Status
    virtual void Status() = 0;
    
    // Notifier for receive complete callback
    void RegisterListenerCB(const std::function<void(CAN_msg_t&)> &recv_cb);

protected:

    // Init/Start RX Thread
    bool StartReceiveThread();

    // Calc Bit Timing Helper Function
    bool CalculateTimings();

    // Config
    Config_t config_;

private:

    void ReceiveTask();

    std::thread rx_thread_;

    std::vector<std::function<void(CAN_msg_t&)>> rx_listeners_;

};

#endif // CAN_CANDEVICE_H_