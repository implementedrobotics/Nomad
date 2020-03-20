/*
 * SerialHandler.cpp
 *
 *  Created on: March 19, 2020
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

// Primary Include
#include "SerialHandler.h"

// C System Files

// C++ System Files

// Project Includes
#include "mbed.h"
#include "rtos.h"
#include "motor_controller_interface.h"

uint32_t packet_byte_read_count = 0;
uint32_t rx_in = 0;
int32_t header_begin = 0;
int32_t header_pos = 0;
uint32_t header_size = 5;
const uint32_t buffer_size = 1024;
char rx_buffer[buffer_size];
char header_buffer[5];
const char header[] = "\x01\x02\x03\x04\x05";
bool packet_ready = false;
packet_read_type_t packet_status = WAIT_PACKET;
Packet_t packet;
std::vector<Packet_t> packet_buffer;


#define CONNECT_COMMAND 0x01

void comms_thread_entry()
{
    // Serial Handler
    Serial serial(PA_2, PA_3);
    serial.baud(921600); // set serial baud rate

    SerialHandler *serial_handler = new SerialHandler(&serial);

    // TODO: Print Speed/Rate
    // TODO: Move Encoder Debug Print Here
    packet_buffer.reserve(10);
    while (1)
    {
        if(packet_buffer.size() > 0)
        {
            // Got a new packet
            //printf("Got a new packet!\n\r");
            Packet_t packet_ = packet_buffer.back();
            packet_buffer.pop_back();
            //printf("Packet Length: %d/%d\n\r", packet_.length, (uint32_t)packet_.data);
            Command_t *command = (Command_t *)packet_.data;
            if(command->command == CONNECT_COMMAND)
            {
                Reponse_t response;
                response.value = 0x02;
                uint8_t *buffer = (uint8_t *)&response;
                for(uint32_t i = 0; i < sizeof(response); i++)
                {
                    serial_handler->GetSerial()->putc(buffer[i]);
                }
            }
            
        }
        osDelay(500);
    }
}



SerialHandler::SerialHandler(Serial *uart)
{
    serial_ = uart;                        // UART Handler
    serial_->attach(callback(this, &SerialHandler::Interrupt)); // Attach Serial Interrupt
}

void SerialHandler::Interrupt()
{
    while (serial_->readable())
    {
        switch(packet_status)
        {
            case WAIT_PACKET:
                header_buffer[header_pos] = serial_->getc();

                header_begin = ((header_pos + 1) - header_size);
                if(header_begin < 0)
                {
                    header_begin = header_pos + 1;
                }

                if(header_buffer[header_begin] == header[0]
                && header_buffer[(header_begin+1) % header_size] == header[1]
                && header_buffer[(header_begin+2) % header_size] == header[2]
                && header_buffer[(header_begin+3) % header_size] == header[3]
                && header_buffer[(header_begin+4) % header_size] == header[4]
                )
                {
                    packet_ready = false;
                    packet_status = READ_PACKET;
                    break;
                }
                header_pos = (header_pos + 1) % header_size;
                break;

            case READ_PACKET:
                
                if(rx_in < 4)
                {
                    rx_buffer[rx_in++] = serial_->getc();
                    break;
                }
                else if(rx_in == 4)
                {
                    std::memcpy(&packet.length, rx_buffer, sizeof(uint32_t));
                    packet_byte_read_count = 0;
                    rx_in++;
                }
                else
                {
                    packet.data[packet_byte_read_count++] = serial_->getc();
                    rx_in++;
                    
                }

                if(packet_byte_read_count == packet.length)
                {
                    // Add Packet to Buffer
                    packet_status = WAIT_PACKET;
                    packet_buffer.push_back(packet);
                    packet_ready = true;
                    rx_in = 0;
                    header_pos = 0;
                    header_begin = 0;
                }
                else if(rx_in > buffer_size) // Overflowed
                {
                    packet_status = WAIT_PACKET;
                    rx_in = 0;
                    packet_byte_read_count = 0;
                    header_pos = 0;
                    header_begin = 0;
                }
                break;
            default:
                break;
        }
    }
}