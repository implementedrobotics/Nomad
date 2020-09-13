/*
 * shared.h
 *
 *  Created on: September 9, 2020
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

#ifndef SHARED_H_
#define SHARED_H_

#ifdef __cplusplus
extern "C" {
#endif

#define RX_DMA_BUFFER_SIZE 64
#define TX_DMA_BUFFER_SIZE 64

#include "cmsis_os2.h"
osThreadId_t comms_task_id;   // Communications Task ID
osThreadId_t control_task_id; // Motor Control Task ID

osMessageQueueId_t uart_rx_dma_queue_id; // Message Queue to receive UART data
osMessageQueueId_t uart_tx_dma_queue_id; // Message Queue to transmit UART data

uint8_t uart_rx_dma_buffer[RX_DMA_BUFFER_SIZE];

#ifdef __cplusplus
}
#endif

#endif // SHARED_H_