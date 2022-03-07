  
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


// C System Files
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

// C++ System Files
#include <iostream>
#include <cmath>


// Third Party Includes
#include <libpcanfd.h>

// Project Includes
#include <CAN/CANDevice.h>

void CANDevice::ReceiveTask()
{
    CAN_msg_t msg;
    while (1)
    {
        if (Receive(msg))
        {
            for(auto listener : rx_listeners_) // Pass to any registered listeners
            {
                listener(msg);
            }
        }
        //usleep(10);
    }
}

// CANDevice::CANDevice()
// {
// }

CANDevice::~CANDevice()
{
    if(rx_thread_.joinable())
        rx_thread_.detach();
}

bool CANDevice::CalculateTimings()
{
    // Reset Timings Flag
    bool timings_valid_ = false;

    // Get CAN Clock
    float f_clkcan = config_.clock_freq;
    
    // Bit time for desired rate
    float bit_time = 1.0f / config_.bitrate;

    int t_sync = 1; // 1 TQ for Sync

    // TODO: Support Propagation Delays?
    // uint_t t_prop = 250; // propagation delay ns
    uint16_t t_seg1 = 0;
    uint16_t t_seg2 = 0;

    // Compute Nominal Bitrate Timings
    uint16_t prescaler = 0;
    for(prescaler = 1; prescaler < kMaxNominalPrescaler; prescaler++)
    {
        // How many Time Quanta in this prescaler?
        float tq = static_cast<float>(prescaler) / f_clkcan;

        // Number of Time Quanta per bit
        float num_tq = bit_time / tq;

        // Check Whole Time Quanta
        if(std::fmod(num_tq, 1) != 0)
            continue;

        t_seg1 = num_tq * config_.sample_point - t_sync;
        t_seg2 = num_tq - t_seg1 - t_sync;

        // Verify Time Segment Range 1
        if(t_seg1 < kMinNominalTimeSeg || t_seg1 > kMaxNominalTimeSeg1)
        {
            // Try Next Prescaler
            continue;
        }

        // Verify Time Segment Range 2 
        if(t_seg2 < kMinNominalTimeSeg || t_seg2 > kMaxNominalTimeSeg2)
        {
            // Try Next Prescaler
            continue;
        }

        // Update CAN Config
        config_.brp = prescaler;
        config_.tseg1 = t_seg1;
        config_.tseg2 = t_seg2;
        config_.sjw = t_seg2;
        config_.tq = static_cast<float>(tq * 1e9);

        timings_valid_ = true;
        break;
    }

    // Check Valid Nominal Bitrate
    if(!timings_valid_)
        return false;

    // Bit time for desired data rate
    bit_time = 1.0f / config_.d_bitrate;

    // TODO: Support Propagation Delays?
    t_seg1 = 0;
    t_seg2 = 0;

    // Update Valid Return
    timings_valid_ = false;

    // Compute Data Bitrate Timings
    prescaler = 0;
    for(prescaler = 1; prescaler < kMaxDataPrescaler; prescaler++)
    {
        // How many Time Quanta in this prescaler?
        float tq = static_cast<float>(prescaler) / f_clkcan;

        // Number of Time Quanta per bit
        float num_tq = bit_time / tq;

        // Check Whole Time Quanta
        if(std::fmod(num_tq, 1) != 0)
            continue;

        t_seg1 = num_tq * config_.d_sample_point - t_sync;
        t_seg2 = num_tq - t_seg1 - t_sync;

        // Verify Time Segment Range 1
        if(t_seg1 < kMinDataTimeSeg || t_seg1 > kMaxDataTimeSeg1)
        {
            // Try Next Prescaler
            continue;
        }

        // Verify Time Segment Range 2 
        if(t_seg2 < kMinDataTimeSeg || t_seg2 > kMaxDataTimeSeg2)
        {
            // Try Next Prescaler
            continue;
        }

        // Update CAN Config
        config_.d_brp = prescaler;
        config_.d_tseg1 = t_seg1;
        config_.d_tseg2 = t_seg2;
        config_.d_sjw = t_seg2;

        timings_valid_ = true;
        break;
    }
    return timings_valid_;
}

bool CANDevice::StartReceiveThread()
{
    // TODO: Thread Affiniy?
    std::cout << "[INFO]: CANDevice Receive Thread Started!" << std::endl;
    rx_thread_ = std::thread(&CANDevice::ReceiveTask, this);
    return true;
}
void CANDevice::RegisterListenerCB(const std::function<void(CAN_msg_t&)> &recv_cb)
{
    //std::cout << "Registering Callback" << std::endl;
    rx_listeners_.push_back(recv_cb);
}
