  
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
    // TODO: Calculate Timings Instead?
    //CalculateTimings();

    fd_ = pcanfd_open(device_id.c_str(), OFD_BITRATE | OFD_SAMPLEPT | OFD_DBITRATE | OFD_SAMPLEPT | OFD_CLOCKHZ /*| OFD_NONBLOCKING*/ | PCANFD_INIT_FD, config.bitrate, (int)config.sample_point * 10000, config.d_bitrate, (int)config.d_sample_point * 10000, config.clock_freq);
    //fd_ = pcanfd_open(device_id.c_str(), OFD_BITRATE | OFD_BRPTSEGSJW | OFD_DBITRATE | OFD_BRPTSEGSJW | OFD_CLOCKHZ | PCANFD_INIT_FD, 1, 50, 29, 10, 1, 8, 7, 12, config.clock_freq);
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

    int rx_error = pcanfd_recv_msg(fd_, &pcan_msg);
    if (rx_error)
    {
        // TODO: Log Received Errors??
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