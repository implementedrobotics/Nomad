  
/*
 * PCANDevice.cpp
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
 */

// C System Files
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

// C++ System Files
#include <iostream>

// Third Party Includes
#include <libpcanfd.h>

// Project Includes
#include <CAN/PCANDevice.h>


PCANDevice::PCANDevice() :  CANDevice(), fd_(-1)
{
}

bool PCANDevice::Open(const std::string &device_id, Config_t &config, bool bUseRXThread)
{
    // Copy Config
    config_ = config;

    // TODO: Validate things?
    if (config_.mode_fd == 0)
    {
        perror("[ERROR]: PCANDevice::Open(): Only CAN FD Ddevices Supported Currently.");
        return false;
    }

    if(!CalculateTimings())
        return false;

    //printf("Timings: %d %d %d %d %d\n", config_.tq,config_.brp,config_.tseg1,config_.tseg2,config_.sjw);
    //printf("Data Timings: %d %d %d %d %d\n", config_.tq,config_.d_brp,config_.d_tseg1,config_.d_tseg2,config_.d_sjw);
    //fd_ = pcanfd_open(device_id.c_str(), OFD_BITRATE | OFD_SAMPLEPT | OFD_DBITRATE | OFD_SAMPLEPT | OFD_CLOCKHZ /*| OFD_NONBLOCKING */| PCANFD_INIT_FD, config.bitrate, (int)config.sample_point * 10000, config.d_bitrate, (int)config.d_sample_point * 10000, config.clock_freq);
    fd_ = pcanfd_open(device_id.c_str(), OFD_BITRATE | OFD_BRPTSEGSJW | OFD_DBITRATE | OFD_BRPTSEGSJW | OFD_CLOCKHZ | /*OFD_NONBLOCKING |*/ PCANFD_INIT_FD | PCANFD_INIT_BUS_LOAD_INFO, config_.brp,
                      config_.tseg1,
                      config_.tseg2,
                      config_.sjw,
                      config_.d_brp,
                      config_.d_tseg1,
                      config_.d_tseg2,
                      config_.d_sjw,
                      config.clock_freq);
    if (fd_ < 0)
    {
        perror("[ERROR]: PCANDevice::Open: Failed to Open PCANFD.");
        return false;
    }

    // Clear Filters
    ClearFilters();
    
    // Start RX Thread
    if(bUseRXThread)
        return StartReceiveThread();

    return true;
}

void PCANDevice::Status()
{
    // Check State
    struct pcanfd_state state;
    pcanfd_get_state(fd_, &state);

    float bus_load = state.bus_load/65536.0f * 100;
    printf("\nCAN BUS State: ");
    if (state.bus_state == PCANFD_ERROR_ACTIVE)
        printf("ACTIVE");
    else if (state.bus_state == PCANFD_ERROR_WARNING)
        printf("WARNING");
    else if (state.bus_state == PCANFD_ERROR_PASSIVE)
        printf("PASSIVE");
    else if (state.bus_state == PCANFD_ERROR_BUSOFF)
        printf("BUS OFF");
    printf("\n-----------------------------------------------\nChannel: %d\nBus Load: %.2f%%\n\rTX Sent/Error: %d/%d\tRX Received/Error: %d/%d\n", 
    state.channel_number, bus_load, state.tx_frames_counter, state.tx_error_counter, state.rx_frames_counter, state.rx_error_counter);
    printf("Tx/Rx Pending: %d/%d\n-----------------------------------------------\n\n", state.tx_pending_msgs, state.rx_pending_msgs);

    // TODO: Auto Bus Restart?
}

bool PCANDevice::Send(uint32_t dest_id, uint8_t *data, uint16_t length)
{
    if(length > 64)
    {
        std::cout << "[ERROR]: PCANDevice::Send(): Invalid Message Length" << std::endl;
        return false;
    }

    // TODO: Support non FD Messages?  For now ALWAYS FD + BRS, and ALWAYS standard length

    // Setup PCANFD Send Format
    struct pcanfd_msg pcan_msg;
    memset(&pcan_msg, 0, sizeof(pcan_msg));
    pcan_msg.type = PCANFD_TYPE_CANFD_MSG;
    pcan_msg.id = dest_id;
    pcan_msg.data_len = length;
    pcan_msg.flags = PCANFD_MSG_STD | PCANFD_MSG_BRS;

    // Copy Message Data to PCANFD
    memcpy(pcan_msg.data, data, length);

    // Send Message
    int tx_error = pcanfd_send_msg(fd_, &pcan_msg);
    if (tx_error)
    {
      //  perror("[ERROR]: PCANDevice::Send(): Failed to Send CAN Message.");
        return false;
    }

    return true;
}

bool PCANDevice::Send(CAN_msg_t &msg)
{
    if(msg.length > 64)
    {
        std::cout << "[ERROR]: PCANDevice::Send(): Invalid Message Length" << std::endl;
        return false;
    }

    // TODO: Support non FD Messages?  For now ALWAYS FD + BRS, and ALWAYS standard length

    // Setup PCANFD Send Format
    struct pcanfd_msg pcan_msg;
    memset(&pcan_msg, 0, sizeof(pcan_msg));
    pcan_msg.type = PCANFD_TYPE_CANFD_MSG;
    pcan_msg.id = msg.id;
    pcan_msg.data_len = msg.length;
    pcan_msg.flags = PCANFD_MSG_STD | PCANFD_MSG_BRS;

    // Copy Message Data to PCANFD
    memcpy(pcan_msg.data, msg.data, msg.length);

    // Send Message
    int tx_error = pcanfd_send_msg(fd_, &pcan_msg);
    if (tx_error)
    {
        //perror("[ERROR]: PCANDevice::Send(): Failed to Send CAN Message.");
        return false;
    }

    return true;
}

bool PCANDevice::Receive(CAN_msg_t &msg)
{
    struct pcanfd_msg pcan_msg;
    //auto start_time = std::chrono::high_resolution_clock::now();
    int rx_error = pcanfd_recv_msg(fd_, &pcan_msg);
    //auto time_now = std::chrono::high_resolution_clock::now();
    //auto total_elapsed = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
    //std::cout << "Read Duration: " << total_elapsed << "us" << std::endl;

   // std::cout << "RECEIVED: TYPE: " << pcan_msg.type << std::endl;
    if (rx_error)
    {
        // TODO: Log Received Errors??
        //std::cout << "FALSE: " << rx_error << std::endl;
        return false;
    }

    // TODO: Handle Status/Error Messages somehow?  For now skip
    // TODO: Support other messages. CAN FD ONLY!
    if(pcan_msg.type != PCANFD_TYPE_CANFD_MSG)
    {
        return false;
    }

    msg.id = pcan_msg.id;
    msg.length = pcan_msg.data_len;
    memcpy(msg.data, pcan_msg.data, pcan_msg.data_len);
    return true;
}

bool PCANDevice::Close()
{
    if (fd_ != -1) // Open?
    {
        pcanfd_close(fd_); //Close it
    }
    else
        return false; // Was already open

    return true; // Successful Close
}

// Filters
bool PCANDevice::AddFilter(uint32_t from, uint32_t to)
{
    struct pcanfd_msg_filter filter;
    filter.id_from = from;
    filter.id_to = to;
    filter.msg_flags = PCANFD_MSG_STD;
    int filter_error = pcanfd_add_filter(fd_, &filter);
    if (filter_error)
    {
        // TODO: Log Received Errors??
        std::cout << "[ERROR]: PCANDevice::AddFilter(): Error setting filter value!" << std::endl;
        return false;
    }
    return true;
}

// Filtering Clear
bool PCANDevice::ClearFilters()
{
    pcanfd_del_filters(fd_);
    return true;
}