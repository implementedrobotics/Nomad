/*
 * DRV8323.cpp
 *
 *  Created on: September 23, 2020
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
#include "DRV8323.h"

// C System Files

// C++ System Files

// Project Includes

DRV8323::DRV8323(SPIDevice *spi, GPIO_t enable_pin, GPIO_t nFault_pin) : spi_(spi), enable_(enable_pin), nFault_(nFault_pin)
{

}

uint16_t DRV8323::Init()
{
    // Enable DRV
    LL_GPIO_SetOutputPin(enable_.port, enable_.pin);
    
    // Read Fault Register
    // spi_->Select();
    // uint16_t val = (1 << 15) | 0x0;
    // uint16_t status_reg = spi_->TransmitReceive16(val);
    // spi_->Deselect();
    // return status_reg;
    return ReadRegister(OCPControl);
}

void DRV8323::EnableDriver()
{
    // Enable DRV
    LL_GPIO_SetOutputPin(enable_.port, enable_.pin);
}

void DRV8323::DisableDriver()
{
    // Enable DRV
    LL_GPIO_ResetOutputPin(enable_.port, enable_.pin);
}


uint16_t DRV8323::ReadRegister(uint16_t address)
{
    uint16_t tx_data = (1 << 15) | (address << 11);
    uint16_t rx_data = 0;

    spi_->Select();
    rx_data = spi_->TransmitReceive16(tx_data);
    spi_->Deselect();
    return rx_data;
}

