/*
 * uart.h
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

#ifndef CORE_PERIPHERAL_UART_H_
#define CORE_PERIPHERAL_UART_H_

#ifdef __cplusplus
extern "C"
{
#endif

#define RX_DMA_BUFFER_SIZE 64
#define TX_DMA_BUFFER_SIZE 64

#define RX_STACK_SIZE 512
#define TX_STACK_SIZE 512

#include <cmsis_os2.h>

// Define some callbacks
typedef void (*uart_rx_cb)(const uint8_t *data, size_t length);

osThreadId_t uart_rx_thread_id;   // UART Receive Task ID
osThreadId_t uart_tx_thread_id;   // UART Transmit Task ID

extern osMessageQueueId_t uart_rx_queue_id; // Message Queue to receive UART data
extern osMessageQueueId_t uart_tx_queue_id; // Message Queue to transmit UART data

uint8_t uart_rx_buffer[RX_DMA_BUFFER_SIZE]; // RX Receive Buffer

void init_uart_threads(void *arg);
void init_uart_rx_thread(); // Thread for UART Receive
void init_uart_tx_thread(); // Thread for UART Transmit

void uart_rx_buffer_process(); // Helper functions.

uint32_t uart_send_data(const uint8_t *data, size_t length); // Send bytes over UART
uint32_t uart_send_str(const char *data); // Send string over UART

void register_rx_callback(uart_rx_cb rx_callback); // Receive callback for received bytes
void echo_rx(const uint8_t *data, size_t length);  // Default callback loopback echo


#ifdef __cplusplus
}
#endif
#endif // CORE_PERIPHERAL_UART_H_