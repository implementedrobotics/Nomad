
/*
 * spi.cpp
 *
 *  Created on: September 21, 2020
 *      Author: Quincy Jones
 *
 * Copyright (c) <2020> <Quincy Jones - quincy@implementedrobotics.com/>
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software
 * is furnished to do so, subject to the following conditions
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
#include <Peripherals/spi.h>

// C System Files

// C++ System Files

// Project Includes
#include<main.h>

SPIDevice::SPIDevice(SPI_TypeDef *SPI, GPIO_t mosi_pin, GPIO_t miso_pin, GPIO_t nss_pin) : SPI_(SPI), mosi_(mosi_pin), miso_(miso_pin), nss_(nss_pin)
{
}

void SPIDevice::Flush()
{
    // Clear RX Buffer
    while (LL_SPI_IsActiveFlag_RXNE(SPI_))
    {
        LL_SPI_ReceiveData8(SPI_);
    }

    while (!LL_SPI_IsActiveFlag_TXE(SPI_))
    {
    }
}

void SPIDevice::Transmit8(uint8_t send_byte)
{
    // Transmit
    while (!LL_SPI_IsActiveFlag_TXE(SPI_));
    LL_SPI_TransmitData8(SPI_, send_byte);
}

void SPIDevice::Transmit16(uint16_t send_bytes)
{
    // Transmit
    while (!LL_SPI_IsActiveFlag_TXE(SPI_));
    LL_SPI_TransmitData16(SPI_, send_bytes);
}

uint16_t SPIDevice::Receive8(void)
{
    // Flush FIFOs
    // Wait for receive buffer to fill
    while (!LL_SPI_IsActiveFlag_RXNE(SPI_));

    // Return receive buffer
    return LL_SPI_ReceiveData8(SPI_);
}

uint16_t SPIDevice::Receive16(void)
{
    // Flush FIFOs

    // Wait for receive buffer to fill
    while (!LL_SPI_IsActiveFlag_RXNE(SPI_));

    // Return receive buffer
    return LL_SPI_ReceiveData16(SPI_);
}

uint8_t SPIDevice::TransmitReceive8(uint8_t send_byte)
{
    // Flush FIFOs
    //Flush();

    // Transmit
    while (!LL_SPI_IsActiveFlag_TXE(SPI_));
    LL_SPI_TransmitData8(SPI_, send_byte);

    // Wait for receive buffer to fill
    while (!LL_SPI_IsActiveFlag_RXNE(SPI_));

    // Return receive buffer
    return LL_SPI_ReceiveData8(SPI_);
}

uint16_t SPIDevice::TransmitReceive16(uint16_t send_bytes)
{
    // Flush FIFOs
    //Flush();
    
    // Transmit
    while (!LL_SPI_IsActiveFlag_TXE(SPI_));
    LL_SPI_TransmitData16(SPI_, send_bytes);

    // Wait for receive buffer to fill
    while (!LL_SPI_IsActiveFlag_RXNE(SPI_));

    
    // Return receive buffer
    return LL_SPI_ReceiveData16(SPI_);
}

// Transmit/Receive Large Buffers
void SPIDevice::TransmitReceive(uint8_t *tx_buffer, size_t tx_length, uint8_t *rx_buffer, size_t rx_length)
{
    Transmit(tx_buffer, tx_length);
    Receive(rx_buffer, rx_length);
}
void SPIDevice::Transmit(uint8_t *data, size_t length)
{
    // TODO: Should transfer 16-bits and then 8-bit transfer any remainder
    // Flush FIFOs
    //Flush();

    for (; length > 0; --length, ++data)
    {
        // Transmit
        while (!LL_SPI_IsActiveFlag_TXE(SPI_));
        LL_SPI_TransmitData8(SPI_, *data);
    }
}
void SPIDevice::Receive(uint8_t *data, size_t length)
{
    // TODO: Should transfer 16-bits and then 8-bit transfer any remainder
    // Flush FIFOs
    //Flush();

    for (; length > 0; --length, ++data)
    {
        // Transmit
        while (!LL_SPI_IsActiveFlag_TXE(SPI_));
        LL_SPI_TransmitData8(SPI_, 0xFF);

        // Wait for receive buffer to fill and receive data
        while (!LL_SPI_IsActiveFlag_RXNE(SPI_));
        *data = LL_SPI_ReceiveData8(SPI_);
    }
}