  
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


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <CAN/CANDevice.h>
#include <linux/can/raw.h>


CANDevice::CANDevice() :  socket_(-1)
{
}

bool CANDevice::Open(const std::string &canid, int32_t timeout)
{
    rx_timeout_ = timeout;
    int mtu, enable_canfd = 1;
	struct sockaddr_can addr;
	struct ifreq ifr;

	if((socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
	{
		perror("Socket Create Error");
		return false;
	}

	strcpy(ifr.ifr_name, canid.c_str());
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

    // Set timeout for socket receive.  Notice some hanging when this is not set
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = rx_timeout_ * 1000;
    setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof(struct timeval));

    if (bind(socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
	{
		perror("BIND ERROR");
		return false;
	}
    return true;
}

bool CANDevice::Send(CAN_msg_t &msg)
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

    // if (write(socket_, &frame, int(m_socket_mode)) != int(m_socket_mode)) {
    //         perror("write");
    //         return STATUS_WRITE_ERROR;
    //     }

    return true;
}

bool CANDevice::Close()
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

// 	frame.can_id = 0x123;
// 	frame.can_dlc = 5;
// 	//sprintf(frame.data, (unsigned char *)"HELLO");
// 	frame.data[0] = 'H';
// 	frame.data[1] = 'E';
// 	frame.data[2] = 'L';
// 	frame.data[3] = 'L';
// 	frame.data[4] = 'O';

// 	if(write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
// 		perror("WRITE ERROR");
// 		return 1;
// 	}


