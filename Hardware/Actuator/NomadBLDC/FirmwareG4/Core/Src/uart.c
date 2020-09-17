/*
 * uart.cpp
 *
 *  Created on: September 15, 2020
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
#include <Peripherals/uart.h>
#include <Utilities/crc16.h>

// C System Files
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// Project Includes
#include <main.h> // STM32 Driver Includes
// #include "stm32g4xx_ll_usart.h"
// #include "stm32g4xx_ll_dma.h"

#define FRAME_BOUNDARY 0x7E
#define CONTROL_ESCAPE 0X7D
#define ESCAPE_INVERT 0X20

#define PACKET_SIZE_LIMIT 256

uart_rx_cb rx_callback = NULL; // Callback for RX buffer receive bytes
USART_TypeDef *USART_ = NULL;  // USART Peripheral

osMessageQueueId_t uart_rx_queue_id = 0; // RX Queue ID
osMessageQueueId_t uart_tx_queue_id = 0; // TX Queue ID

uint8_t uart_rx_buffer[RX_DMA_BUFFER_SIZE]; // RX Receive Buffer

uint16_t frame_offset;
uint16_t frame_chksum;
uint8_t hdlc_rx_buffer[512]; // Frame buffer.  Support 255
uint8_t hdlc_tx_buffer[512]; // Frame buffer out
uint8_t in_escape;           // Are we currently in escape?

uart_mode_t mode_;

void init_uart_threads(void *arg)
{
    USART_ = (USART_TypeDef *)arg;

    // Create Message Queue
    uart_rx_queue_id = osMessageQueueNew(10, sizeof(void *), NULL);

    // Setup mode.  HDLC Default
    mode_ = HDLC;

    // HDLC Initial
    frame_offset = 0;
    in_escape = 0;

    // Start RX Thread
    const osThreadAttr_t task_rx_attributes = {
        .name = "uart_rx_task",
        .priority = (osPriority_t)osPriorityNormal,
        .stack_size = RX_STACK_SIZE};

    osThreadNew(init_uart_rx_thread, NULL, &task_rx_attributes);

    // // Start TX Thread
    // const osThreadAttr_t task_tx_attributes = {
    //     .name = "uart_tx_task",
    //     .priority = (osPriority_t)osPriorityNormal,
    //     .stack_size = TX_STACK_SIZE};
    // osThreadNew(init_uart_tx_thread, NULL, &task_tx_attributes);

    osThreadExit();
}
void init_uart_rx_thread(void *arg)
{
    void *d; // TODO: Semaphore?

    uart_send_str("Nomad Firmware v2.0 STM32G4 Beta\r\n");

    // Set default callback
    switch (mode_)
    {
    case HDLC:
        register_rx_callback(hdlc_rx);
        uart_send_str("UART MODE: HDLC\r\n");
        break;

    case ASCII:
        register_rx_callback(echo_rx);
        uart_send_str("UART MODE: ASCII\r\n");
        break;

    case BINARY:
        register_rx_callback(echo_rx);
        uart_send_str("UART MODE: BINARY\r\n");
        break;

    default:
        register_rx_callback(echo_rx);
        uart_send_str("UART MODE: UNDEFINED!\r\n");
    }

    for (;;)
    {
        // Wait for QUEUE send from interrupt. // TODO: Semaphore?
        osMessageQueueGet(uart_rx_queue_id, &d, NULL, osWaitForever);

        // Read our receive buffer
        uart_rx_buffer_process();

        (void)d; // Clean "unused" variable compiler warning
    }
}

// void init_uart_tx_thread()
// {
//     void *d = (void *)1;
//     for (;;)
//     {
//         //osMessageQueuePut(uart_rx_queue_id, &d, 0, 0); // Send Data to Queue and Leave w/o timeout
//         osDelay(500);
//         //uart_send_str("Hello\r\n");
//     }

// }

void uart_rx_buffer_process()
{
    static size_t old_pos;
    size_t pos;

    // Compute where we are in the DMA buffer
    pos = RX_DMA_BUFFER_SIZE - LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_1);
    if (pos != old_pos) // We have new data in buffer
    {
        if (pos > old_pos) // Standard buffer read no wrapping.
        {
            // We are in "linear" mode
            // Process data directly by subtracting "pointers"
            rx_callback(&uart_rx_buffer[old_pos], pos - old_pos);
        }
        else // We are have wrapped the buffer
        {
            // Process "linear" to end of buffer
            rx_callback(&uart_rx_buffer[old_pos], RX_DMA_BUFFER_SIZE - old_pos);
            // Read rest of bytes from the beginning of the buffer to get the wrapped bytes
            if (pos > 0)
            {
                rx_callback(&uart_rx_buffer[0], pos);
            }
        }
    }
    old_pos = pos; // Update current/old position pointer

    // Check if we moved to end of buffer and need to wrap back to beginning
    if (old_pos == RX_DMA_BUFFER_SIZE)
    {
        old_pos = 0;
    }
}

uint32_t uart_send_data(const uint8_t *data, size_t length)
{
    uint32_t sent_bytes = 0;
    for (; length > 0; --length, ++data)
    {
        LL_USART_TransmitData8(USART_, *data);
        LL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
        while (!LL_USART_IsActiveFlag_TXE(USART_))
        {
        }
        sent_bytes++;
    }
    while (!LL_USART_IsActiveFlag_TC(USART_))
    {
    }
    return sent_bytes;
}

uint32_t uart_send_str(const char *data)
{
    return uart_send_data((uint8_t *)data, strlen(data));
}

uint32_t uart_send_data_hdlc(const uint8_t *packet, size_t length)
{
    if (length >= PACKET_SIZE_LIMIT)
    {
        return 0;
    }

    // Compute CRC16 for Packet
    frame_chksum = crc16_compute(packet, length);

    uint32_t buffer_offset = 0;
    hdlc_tx_buffer[buffer_offset++] = FRAME_BOUNDARY;

    // Process and Escape Packet
    for (uint32_t i = 0; i < length; i++)
    {
        uint8_t data = packet[i];
        if ((data == FRAME_BOUNDARY) || (data == CONTROL_ESCAPE))
        {
            hdlc_tx_buffer[buffer_offset++] = CONTROL_ESCAPE;
            hdlc_tx_buffer[buffer_offset++] = data ^ ESCAPE_INVERT;
        }
        else // Not Escaped
        {
            hdlc_tx_buffer[buffer_offset++] = data;
        }
    }

    // Copy in CRC18
    memcpy(hdlc_tx_buffer + buffer_offset, (uint8_t *)(&frame_chksum), sizeof(uint16_t));

    // Add Frame Boundary
    buffer_offset += 2;
    hdlc_tx_buffer[buffer_offset++] = FRAME_BOUNDARY;

    // Send it
    return uart_send_data(hdlc_tx_buffer, buffer_offset);
}

// Register Receive Callback
void register_rx_callback(uart_rx_cb callback)
{
    rx_callback = callback;
}

// Default RX Byte Handler.  Simple loopback echo
void echo_rx(const uint8_t *data, size_t length)
{
    uart_send_data(data, length);
}

// Default HDLC Handler.  Frame pack for HDLC and send
void hdlc_rx(const uint8_t *data, size_t length)
{
    for (; length > 0; --length, ++data)
    {
        uint8_t byte = *data;
        if (byte == FRAME_BOUNDARY && !in_escape)
        {
            // Check for End Frame + Validity
            if (frame_offset >= 2) // Need atleast 3 bytes for a valid frame, (BEGIN, CMD, LENGTH)
            {
                // Command = receive_buffer_[0]
                // Payload Length = receive_buffer_[1]
                // Fast early out on packet length
                if ((frame_offset - 4) == hdlc_rx_buffer[1])
                {
                    // Length matches now verify checksum
                    uint16_t sent_chksum = (hdlc_rx_buffer[frame_offset - 1] << 8) | (hdlc_rx_buffer[frame_offset - 2] & 0xff);
                    frame_chksum = crc16_compute(hdlc_rx_buffer, frame_offset - 2);
                    if (frame_chksum == sent_chksum)
                    {
                        // Execute Command Callback
                        //CommandHandler::ProcessPacket(receive_buffer_, frame_offset - 2);
                    }
                }
                // TODO: If invalid do we add support for an ack?
            }
            // Reset and look for next Frame
            frame_offset = 0;
            frame_chksum = 0;
            return;
        }

        // Handle Escape Sequences
        if (in_escape)
        {
            byte ^= ESCAPE_INVERT;
            in_escape = 0;
        }
        else if (byte == CONTROL_ESCAPE)
        {
            in_escape = 1;
            return; // Return here to read real byte on next run
        }

        // Copy to buffer
        hdlc_rx_buffer[frame_offset++] = byte;

        if (frame_offset >= PACKET_SIZE_LIMIT) // Overflow Packet Limit,
        {
            frame_offset = 0; // Reset
            frame_chksum = 0;
        }
    }
}
