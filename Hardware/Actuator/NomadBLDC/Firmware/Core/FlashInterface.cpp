/*
 * FlashInterface.cpp
 *
 *  Created on: August 29, 2019
 *      Author: Quincy Jones
 *
 * Copyright (c) <2019> <Quincy Jones - quincy@implementedrobotics.com/>
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
#include "FlashInterface.h"

// C System Files

// C++ System Files

// Project Includes
#include "mbed.h"

FlashInterface::FlashInterface() : ready_(false), mode_(FlashInterface::READ), sector_(0)
{
}

FlashInterface::~FlashInterface()
{
    // Close if opened
    Close();
}
bool FlashInterface::Open(uint32_t sector, mode_type_t mode)
{

    // Check for Valid Sector
    if (sector > 7) // 7 Max Sectors on the 446RE
        return false;
    else if (ready_ == true) // Flash opened already, can't open again!
        return false;

    // Set Sector Address/Base Values
    sector_ = sector;
    base_ = __SECTOR_ADDRS[sector];

    mode_ = mode;                       // Set Mode
    if (mode_ == FlashInterface::WRITE) // Open for write
    {
        FLASH_Unlock(); // Unlock Flash
        FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
        FLASH_EraseSector(__SECTORS[sector_], VoltageRange_3); // On writes we need to erase the sector
    }

    ready_ = true;
    return true; // Sucessfully Opened
}

bool FlashInterface::Write(uint32_t index, uint8_t *buffer, size_t length)
{
    if (!ready_ || mode_ != FlashInterface::WRITE) // Need to be Ready and in Write Mode
        return false;

    uint32_t write_addr = base_ + index; // Set Write Buffer Offset
    for (uint32_t i = 0; i < length; i++) // Loop buffer and write out in bytes
    {
        FLASH_ProgramByte(write_addr, ((uint8_t *)buffer)[i]);
        write_addr++;
    }
    return true;
}

bool FlashInterface::Read(uint32_t index, void *buffer, size_t length)
{
    if (!ready_) // Can always read even in write mode...
        return false;

    memcpy(buffer, (uint8_t *)(base_ + index), sizeof(uint8_t) * length);

    return true;
}

bool FlashInterface::Close()
{
    if (!ready_) // If not opened nothing to do, return
        return false;

    if (mode_ == FlashInterface::WRITE) // Lock Flash if we were in write mode
        FLASH_Lock();

    mode_ = FlashInterface::READ; // Default in read mode
    ready_ = false;

    return true;
}

FlashInterface &FlashInterface::Instance()
{
    static FlashInterface instance;
    return instance;
}