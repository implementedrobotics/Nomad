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

// C System Files

// C++ System Files

// Project Includes
#include <main.h> // STM32 Driver Includes
// #include "stm32g4xx_ll_usart.h"
// #include "stm32g4xx_ll_dma.h"

// HDLC Handler
//HDLCHandler hdlc;

// Comms Event Loops

// C interface
// #include "main.h"
// #include "cmsis_os.h"
// #include "shared.h"
// #include "thread_interface.h"


// SerialHandler::SerialHandler()
// {
// }

// void SerialHandler::SetUSART(USART_TypeDef *USART)
// {
//     USART_ = USART;
// }

// void SerialHandler::SendString(const std::string &str)
// {
//     SendData((uint8_t *)str.c_str(), str.length());
// }
// void SerialHandler::SendData(const uint8_t* data, size_t length)
// {
//     for(; length > 0; --length, ++data) {
//         LL_USART_TransmitData8(USART_, *data);
//         while(!LL_USART_IsActiveFlag_TXE(USART_)) {}
//     }
//     while(!LL_USART_IsActiveFlag_TC(USART_)) {}
// }

// // Singleton Insance
// SerialHandler &SerialHandler::Instance()
// {
//     static SerialHandler instance;
//     return instance;
// }

// void uart_rx_dma_thread()
// {
//     // HDLC Handler    
//     //HDLCHandler hdlc;
//     void *d;

//     LL_GPIO_SetOutputPin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
//     /* Notify user to start sending data */
//     //usart_send_string("USART DMA example: DMA HT & TC + USART IDLE LINE IRQ + RTOS processing\r\n");
//     //usart_send_string("Start sending data to STM32\r\n");
//     // Reset Message Queues.  Not sure if this is actually necessary
//     osMessageQueueReset(uart_rx_dma_queue_id);

//     SerialHandler::Instance().SetUSART(USART2);
//     SerialHandler::Instance().SendString("Hello\r\n");
//     SerialHandler serial = SerialHandler::Instance();

//     for (;;)
//     {
//        // osMessageQueueGet(uart_rx_dma_queue_id, &d, NULL, osWaitForever);

//         /* Block thread and wait for event to process USART data */
//         //osMessageQueueGet(uart_rx_dma_queue_id, &d, NULL, osWaitForever);
//         LL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
//         //LL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);

//         osDelay(500);

//         serial.SendString("Hello\r\n");
        

//         /* Simply call processing function */
//         //usart_rx_check();
        
//         (void)d;
//     }
// }

void init_uart_rx_thread()
{
    void *d; // TODO: Semaphore?

    for(;;)
    {
        // Wait for QUEUE send from interrupt. // TODO: Semaphore?
        osMessageQueueGet(uart_rx_queue_id, &d, NULL, osWaitForever);

        // Read our receive buffer
        uart_rx_buffer_process();

        (void)d; // Clean "unused" variable compiler warning
    }
}

void init_uart_tx_thread()
{

}

// TODO: Should have callback for byte process function?
void uart_rx_buffer_process()
{
    static size_t old_pos;
    size_t pos;

    /* Calculate current position in buffer */
    pos = RX_DMA_BUFFER_SIZE - LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_1);
    if (pos != old_pos) {                       /* Check change in received data */
        if (pos > old_pos) {                    /* Current position is over previous one */
            /* We are in "linear" mode */
            /* Process data directly by subtracting "pointers" */
            //usart_process_data(&usart_rx_dma_buffer[old_pos], pos - old_pos);
        } else {
            /* We are in "overflow" mode */
            /* First process data to the end of buffer */
            //usart_process_data(&usart_rx_dma_buffer[old_pos], RX_DMA_BUFFER_SIZE - old_pos);
            /* Check and continue with beginning of buffer */
            if (pos > 0) {
                //usart_process_data(&usart_rx_dma_buffer[0], pos);
            }
        }
    }
    old_pos = pos;                              /* Save current position as old */

    /* Check and manually update if we reached end of buffer */
    if (old_pos == RX_DMA_BUFFER_SIZE) {
        old_pos = 0;
    }
}