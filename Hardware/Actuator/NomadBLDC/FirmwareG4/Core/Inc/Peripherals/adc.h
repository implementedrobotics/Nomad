// /*
//  * adc.h
//  *
//  *  Created on: August 27, 2019
//  *      Author: Quincy Jones
//  *
//  * Copyright (c) <2019> <Quincy Jones - quincy@implementedrobotics.com/>
//  * Permission is hereby granted, free of charge, to any person obtaining a
//  * copy of this software and associated documentation files (the "Software"),
//  * to deal in the Software without restriction, including without limitation
//  * the rights to use, copy, modify, merge, publish, distribute, sublicense,
//  * and/or sell copies of the Software, and to permit persons to whom the Software
//  * is furnished to do so, subject to the following conditions:
//  * The above copyright notice and this permission notice shall be included in all
//  * copies or substantial portions of the Software.
//  * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
//  * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
//  * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//  * 
//  */

// #ifndef CORE_PERIPHERAL_SPI_H_
// #define CORE_PERIPHERAL_SPI_H_

// // C System Files

// // C++ System Files

// // Project Includes
// #include "stm32g4xx_ll_spi.h"
// #include <Peripherals/gpio.h>

// class SPIDevice
// {

// public:

//     SPIDevice(SPI_TypeDef *SPI, GPIO_t mosi_pin, GPIO_t miso_pin, GPIO_t nss_pin);

//     // Chips Select/Deselects
//     inline void Select() { LL_GPIO_ResetOutputPin(nss_.port, nss_.pin); }
//     inline void Deselect() { LL_GPIO_SetOutputPin(nss_.port, nss_.pin); }
//     inline void Enable() { LL_SPI_Enable(SPI_); }
//     inline void Disable() { LL_SPI_Disable(SPI_); }
    
//     // Flush FIFOs
//     void Flush();

//     // Receive  8 bitsfrom SPI (Will send dummy bytes 0xFF to shift in data)
//     uint16_t Receive8(void);

//     // Receive 16 bits from SPI (Will send dummy bytes 0xFFFF to shift in data)
//     uint16_t Receive16(void);

//     // Receive 16 bits from SPI (Transmit send_bytes)
//     uint16_t TransmitReceive16(uint16_t send_bytes);

//     // Receive 8 bits from SPI (Transmit send_bytes)
//     uint8_t TransmitReceive8(uint8_t send_bytes);

//     // Transmit/Receive Large Buffers
//     void TransmitReceive(uint8_t *tx_buffer, size_t tx_length, uint8_t *rx_buffer, size_t rx_length);
//     void Transmit(uint8_t *tx_buffer, size_t tx_length);
//     void Receive(uint8_t *rx_buffer, size_t rx_length);

// private:

//     SPI_TypeDef *SPI_;
//     GPIO_t mosi_;
//     GPIO_t miso_;
//     GPIO_t nss_;

// };

// #endif // CORE_PERIPHERAL_SPI_H_