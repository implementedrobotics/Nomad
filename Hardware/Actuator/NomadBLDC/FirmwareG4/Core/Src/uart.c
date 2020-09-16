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
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

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

// void SerialHandler::SendString(const std::string &str)
// {
//     SendData((uint8_t *)str.c_str(), str.length());
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

// }

uart_rx_cb rx_callback = NULL; // Callback for RX buffer receive bytes
USART_TypeDef *USART_ = NULL;  // USART Peripheral

osMessageQueueId_t uart_rx_queue_id = 0;
osMessageQueueId_t uart_tx_queue_id = 0;

void init_uart_threads(void *arg)
{
    USART_ = (USART_TypeDef *)arg;

    // Create Message Queue
    uart_rx_queue_id = osMessageQueueNew(10, sizeof(void *), NULL);

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

    // Set default callback
    register_rx_callback(echo_rx);

    uart_send_str("Nomad Firmware v2.0 STM32G4 Beta\r\n");

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
        while (!LL_USART_IsActiveFlag_TXE(USART_)){}
        sent_bytes++;
    }
    while (!LL_USART_IsActiveFlag_TC(USART_)){}
    return sent_bytes;
}

uint32_t uart_send_str(const char *data)
{
    return uart_send_data((uint8_t*)data, strlen(data));
}

// Register RX Callback
void register_rx_callback(uart_rx_cb callback)
{
    rx_callback = callback;
}

// Default RX Byte Handler.  Simple loopback echo
void echo_rx(const uint8_t *data, size_t length)
{
    uart_send_data(data, length);
}

