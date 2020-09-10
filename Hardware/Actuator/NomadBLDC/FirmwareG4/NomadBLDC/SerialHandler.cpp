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

// HDLC Handler
//HDLCHandler hdlc;

// Comms Event Loops

// C interface
#include "main.h"
#include "cmsis_os.h"
#include "shared.h"
#include "thread_interface.h"

void uart_rx_dma_thread()
{
    // HDLC Handler    
    //HDLCHandler hdlc;
    void *d;

    LL_GPIO_SetOutputPin(GPIOB, GPIO_PIN_10);
    /* Notify user to start sending data */
    //usart_send_string("USART DMA example: DMA HT & TC + USART IDLE LINE IRQ + RTOS processing\r\n");
    //usart_send_string("Start sending data to STM32\r\n");
    // Reset Message Queues.  Not sure if this is actually necessary
    osMessageQueueReset(uart_rx_dma_queue_id);

    for (;;)
    {
        /* Block thread and wait for event to process USART data */
        //osMessageQueueGet(uart_rx_dma_queue_id, &d, NULL, osWaitForever);
        LL_GPIO_TogglePin(GPIOB, GPIO_PIN_10);
        //LL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);

        osDelay(500);

        /* Simply call processing function */
        //usart_rx_check();
        
        (void)d;
    }
}

void uart_tx_dma_thread()
{
    void *d;

    LL_GPIO_SetOutputPin(GPIOB, GPIO_PIN_10);
    /* Notify user to start sending data */
    osMessageQueueReset(uart_tx_dma_queue_id);
    for (;;)
    {
        /* Block thread and wait for event to process USART data */
        //osMessageQueueGet(uart_rx_dma_queue_id, &d, NULL, osWaitForever);
        LL_GPIO_TogglePin(GPIOB, GPIO_PIN_10);
        //LL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);

        osDelay(500);

        /* Simply call processing function */
        //usart_rx_check();

        (void)d;
    }
}
