  
/*
 * SocketCANDevice.cpp
 *
 *  Created on: March 20, 2021
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <iostream>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can/raw.h>
#include <CAN/SocketCANDevice.h>



SocketCANDevice::SocketCANDevice() :  CANDevice(), socket_(-1)
{
}

bool SocketCANDevice::Open(const std::string &device_id, Config_t &config, bool bUseRXThread)
{
   // rx_timeout_ = timeout;
    int mtu, enable_canfd = 1;
	struct sockaddr_can addr;
	struct ifreq ifr;

	if((socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
	{
		perror("Socket Create Error");
		return false;
	}

	strcpy(ifr.ifr_name, device_id.c_str());
	ioctl(socket_, SIOCGIFINDEX, &ifr);

	memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

    if (ioctl(socket_, SIOCGIFMTU, &ifr) < 0)
    {
        perror("SIOCGIFMTU");
        return false;
    }
    /* interface is ok - try to switch the socket into CAN FD mode */
    if (setsockopt(socket_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES,
                   &enable_canfd, sizeof(enable_canfd)))
    {

        perror("here");
        return false;
    }

    // // Set timeout for socket receive.  Notice some hanging when this is not set
    // struct timeval tv;
    // tv.tv_sec = 0;
    // tv.tv_usec = rx_timeout_ * 1000;
    // setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof(struct timeval));

    // if (bind(socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
	// {
	// 	perror("BIND ERROR");
	// 	return false;
    // }
    return true;
}
bool SocketCANDevice::Send(uint32_t dest_id, uint8_t *data, uint16_t length)
{
    if(length > 64)
    {
        std::cout << "[ERROR]: SocketCANDevice::Send(): Invalid Message Length" << std::endl;
        return false;
    }

    // TODO: Support non FD Messages?  For now ALWAYS FD + BRS, and ALWAYS standard length

    // Setup PCANFD Send Format
    struct canfd_frame frame;

    frame.can_id = dest_id;
    frame.len = length;// DLC_LUT[msg.length] << 16;
    memcpy(frame.data, data, length);

    if (write(socket_, &frame, sizeof(struct canfd_frame)) != sizeof(struct canfd_frame))
    {
        perror("WRITE ERROR");
        return false;

    }

    return true;
}

bool SocketCANDevice::Send(CAN_msg_t &msg)
{
    // TODO: Make sure socket is open and can device is valid, etc
    struct canfd_frame frame;
    memset(&frame, 0, sizeof(frame)); /* init CAN FD frame, e.g. LEN = 0 */

    frame.can_id = msg.id;
    frame.len = msg.length;// DLC_LUT[msg.length] << 16;
    memcpy(frame.data, msg.data, msg.length);

    if (write(socket_, &frame, sizeof(struct canfd_frame)) != sizeof(struct canfd_frame))
    {
        perror("WRITE ERROR");
        return false;

    }

    return true;
}

bool SocketCANDevice::Receive(CAN_msg_t &msg)
{
    // TODO: Make sure socket is open and can device is valid, etc
    struct canfd_frame frame;

    // Read in a CAN frame
    auto num_bytes = read(socket_, &frame, CANFD_MTU);
    //std::cout << "READ: " << num_bytes << std::endl;
    if (num_bytes != CAN_MTU && num_bytes != CANFD_MTU)
    {
        //perror("READ ERROR");
        return false;
    }

    //std::cout << "WHO TO: " << pcan_msg.id << std::endl;
    msg.id = frame.can_id;
    msg.length = frame.len;
    memcpy(msg.data, frame.data, frame.len);
    return true;
}

bool SocketCANDevice::Close()
{
    if (socket_ != -1)
    {
        if (close(socket_) < 0)
        {
            perror("ERROR CLOSE");
            return false;
        }
    }
    else
        return false;

    return true;
}




// Filters
bool SocketCANDevice::AddFilter(uint32_t from, uint32_t to)
{
    return true;
}

// Filtering Clear
bool SocketCANDevice::ClearFilters()
{
    return true;
}