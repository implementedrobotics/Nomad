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

#ifndef CORE_PERIPHERAL_UARTNEW_H_
#define CORE_PERIPHERAL_UARTNEW_H_

#include <cmsis_os2.h>
#include <stdlib.h>
#include <stdint.h>
#include <Peripherals/gpio.h>
#include <functional>
#include <main.h> 

class UARTDevice
{

public:

    // Const Statics
    static const uint16_t kBufferSizeRX = 64;
    static const uint16_t kBufferSizeTX = 64;

    static const uint16_t kStackSizeRX = 2048;
    static const uint16_t kStackSizeTX = 1024;

    // Data Type
    // TODO: Should we have Handlers register callbacks or have it be part of the driver
    typedef enum
    {
        ASCII = 0,
        BINARY,
        HDLC
    } UART_DATA_MODE;

    // Signals
    enum Signals
    {
        UART_RX_DATA = (1 << 0)
    };

    // Constructor
    UARTDevice(USART_TypeDef *UART, GPIO_t rx_pin, GPIO_t tx_pin);
    
    // Set Mode
    void SetMode(UART_DATA_MODE mode);

    // Set Baud
    void SetBaud(uint32_t baud);

    // Init UART
    bool Init();

    // Callback function for receive buffer processing
    void RegisterRXCallback(const std::function<void(const uint8_t*, size_t)>& callback);

    // Receive Handler for RX Task Callback
    void ReceiveHandler();

    // Encode UART Packet for HDLC
    uint32_t SendHDLC(const uint8_t *data, size_t length);

    // Send string over UART
    uint32_t SendString(const char *string);

    // Send Raw data over UART
    uint32_t Send(const uint8_t *data, size_t length); // Send bytes over UART

protected:

    // STM32 USART Init Setup
    void USART_Init();

    // Default is to echo
    void ReceiveEcho(const uint8_t *data, size_t length);

    // Handle received bytes when in HDLC mode
    void ReceiveHDLC(const uint8_t *data, size_t length);

private:

    // STM32 USART Device Pointer
    USART_TypeDef *UART_;

    // RX Line GPIO
    GPIO_t rx_pin_;

    // TX Line GPIO
    GPIO_t tx_pin_;

    // Baud Rate
    uint32_t baud_;

    // DMA?
    bool tx_dma_;
    bool rx_dma_;

    // RX Receive Buffer
    uint8_t uart_rx_buffer_[kBufferSizeRX]; 
    size_t old_rx_buffer_pos_;

    // UART Mode
    UART_DATA_MODE mode_;

    // RX Line Activity callback
    std::function<void(const uint8_t*, size_t)> callback_rx_ = [=](const uint8_t*, size_t){};

};


#endif // CORE_PERIPHERAL_UART_H_
